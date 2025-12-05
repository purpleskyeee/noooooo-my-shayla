#include "vex.h"

#include <algorithm>
#include <atomic>

#include "../include/ball-indexer.h"
#include "definitions.h"

namespace {

using ball_indexer::GeometryConfig;
using ball_indexer::Mode;
using ball_indexer::RuntimeConfig;

std::atomic<bool> thread_should_run{false};
std::atomic<Mode> current_mode{Mode::kDisabled};

GeometryConfig geometry_cfg{};
RuntimeConfig runtime_cfg{};

std::atomic<int> stacked_count{0};           // Balls in the vertical tower
std::atomic<bool> storage_slot_full{false};  // First ball stored as the 11th

vex::thread* index_thread = nullptr;

enum class IntakePhase {
  kSeekStorageBall,
  kLoadingStorage,
  kStacking,
  kFull,
};

IntakePhase intake_phase = IntakePhase::kSeekStorageBall;
bool releasing_stored_ball = false;
bool entry_prev = false;
bool top_prev = false;

int mainStackTarget() {
  return std::max(0, runtime_cfg.total_ball_target - runtime_cfg.reserved_storage_slots);
}

bool sensorTriggered(distance& sensor, double threshold_mm) {
  double measurement = sensor.objectDistance(mm);
  return measurement > 0 && measurement <= threshold_mm;
}

void runUpstreamForward(double volts) {
  intake_primary_motor.spin(vex::forward, volts, vex::voltageUnits::volt);
  intake_stage1_motor.spin(vex::forward, volts, vex::voltageUnits::volt);
}

void runUpstreamBackward(double volts) {
  intake_primary_motor.spin(vex::reverse, volts, vex::voltageUnits::volt);
  intake_stage1_motor.spin(vex::reverse, volts, vex::voltageUnits::volt);
}

void stopUpstream(vex::brakeType type) {
  intake_primary_motor.stop(type);
  intake_stage1_motor.stop(type);
}

void runStage2Down(double volts) {
  intake_stage2_motor.spin(vex::reverse, volts, vex::voltageUnits::volt);
}

void runStage2Up(double volts) {
  intake_stage2_motor.spin(vex::forward, volts, vex::voltageUnits::volt);
}

void stopStage2(vex::brakeType type) {
  intake_stage2_motor.stop(type);
}

void ensureThreadStarted() {
  if (index_thread == nullptr) {
    thread_should_run = true;
    index_thread = new vex::thread([]() -> int {
      while (thread_should_run.load()) {
        Mode mode = current_mode.load();

        bool entry_blocked = sensorTriggered(intake_distance, runtime_cfg.entry_sensor_threshold_mm);
        bool storage_blocked = sensorTriggered(storage_distance_sensor, runtime_cfg.storage_sensor_threshold_mm);
        bool top_blocked = sensorTriggered(top_distance_sensor, runtime_cfg.top_sensor_threshold_mm);

        int target_stack = mainStackTarget();

        if (mode == Mode::kIntake) {
          switch (intake_phase) {
            case IntakePhase::kSeekStorageBall:
              if (storage_slot_full.load()) {
                intake_phase = IntakePhase::kStacking;
                break;
              }
              runUpstreamForward(runtime_cfg.intake_feed_voltage);
              stopStage2(vex::brakeType::hold);
              if (entry_blocked) {
                intake_phase = IntakePhase::kLoadingStorage;
              }
              break;

            case IntakePhase::kLoadingStorage:
              runUpstreamForward(runtime_cfg.intake_feed_voltage);
              runStage2Down(runtime_cfg.stage2_push_down_voltage);
              if (storage_blocked) {
                storage_slot_full = true;
                stopStage2(vex::brakeType::hold);
                intake_phase = IntakePhase::kStacking;
              }
              break;

            case IntakePhase::kStacking:
              if (stacked_count.load() >= target_stack) {
                intake_phase = IntakePhase::kFull;
                break;
              }
              runUpstreamForward(runtime_cfg.intake_feed_voltage);
              break;

            case IntakePhase::kFull:
              if (runtime_cfg.intake_hold_voltage > 0) {
                runUpstreamForward(runtime_cfg.intake_hold_voltage);
              } else {
                stopUpstream(vex::brakeType::hold);
              }
              if (stacked_count.load() < target_stack) {
                intake_phase = IntakePhase::kStacking;
              }
              break;
          }

          if (storage_slot_full.load() && intake_phase == IntakePhase::kStacking &&
              stacked_count.load() < target_stack) {
            if (entry_blocked && !entry_prev) {
              int new_count = std::min(target_stack, stacked_count.load() + 1);
              stacked_count = new_count;
              if (new_count >= target_stack) {
                intake_phase = IntakePhase::kFull;
              }
            }
          }

        } else if (mode == Mode::kOuttake) {
          int current_stack = stacked_count.load();
          if (current_stack > 0) {
            runUpstreamBackward(runtime_cfg.outtake_voltage);
            if (top_prev && !top_blocked) {
              stacked_count = std::max(0, current_stack - 1);
            }
          } else {
            stopUpstream(vex::brakeType::coast);
            if (storage_slot_full.load()) {
              runStage2Up(runtime_cfg.stage2_release_voltage);
              releasing_stored_ball = true;
              if (top_blocked) {
                storage_slot_full = false;
              }
            } else {
              if (releasing_stored_ball && !top_blocked && top_prev) {
                releasing_stored_ball = false;
              }
              stopStage2(vex::brakeType::coast);
            }
          }

        } else {
          stopUpstream(vex::brakeType::coast);
          stopStage2(vex::brakeType::coast);
        }

        entry_prev = entry_blocked;
        top_prev = top_blocked;
        vex::wait(10, vex::msec);
      }

      stopUpstream(vex::brakeType::coast);
      stopStage2(vex::brakeType::coast);
      return 0;
    });
  }
}

}  // namespace

namespace ball_indexer {

void configureGeometry(const GeometryConfig& config) {
  geometry_cfg = config;
}

void configureRuntime(const RuntimeConfig& config) {
  runtime_cfg = config;
}

void enable() {
  setMode(Mode::kIntake);
}

void disable() {
  setMode(Mode::kDisabled);
}

void shutdown() {
  if (index_thread != nullptr) {
    thread_should_run = false;
    index_thread->interrupt();
    delete index_thread;
    index_thread = nullptr;
  }
  current_mode = Mode::kDisabled;
  stopUpstream(vex::brakeType::coast);
  stopStage2(vex::brakeType::coast);
}

void setMode(Mode mode) {
  ensureThreadStarted();
  current_mode = mode;
}

Mode getMode() {
  return current_mode.load();
}

void startOuttake() {
  setMode(Mode::kOuttake);
}

void stopOuttake() {
  setMode(Mode::kIntake);
}

void resetCounts() {
  stacked_count = 0;
  storage_slot_full = false;
  intake_phase = IntakePhase::kSeekStorageBall;
  releasing_stored_ball = false;
  entry_prev = false;
  top_prev = false;
}

int stackedBallCount() {
  return stacked_count.load();
}

bool storageBallReady() {
  return storage_slot_full.load();
}

int totalBallCount() {
  return stacked_count.load() + (storage_slot_full.load() ? runtime_cfg.reserved_storage_slots : 0);
}

void setDetectionThreshold(double threshold_mm) {
  runtime_cfg.entry_sensor_threshold_mm = threshold_mm;
}

void setFeedVoltage(double volts) {
  runtime_cfg.intake_feed_voltage = volts;
}

void setHoldVoltage(double volts) {
  runtime_cfg.intake_hold_voltage = volts;
}

bool isEnabled() {
  return current_mode.load() != Mode::kDisabled;
}

}  // namespace ball_indexer
