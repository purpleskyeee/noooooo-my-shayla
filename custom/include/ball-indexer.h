#pragma once

#include <cstdint>

namespace ball_indexer {

enum class Mode {
  kDisabled,
  kIntake,
  kOuttake,
};

struct GeometryConfig {
  double ball_diameter_mm = 82.0;          // Approx. 2.87 in game pieces
  double stage1_sprocket_teeth = 24.0;     // 11W roller sprocket (1:1 on 24T, 600 RPM)
  double stage2_sprocket_teeth = 16.0;     // 5.5W indexing roller
  double stage3_sprocket_teeth = 16.0;     // Storage pusher sprocket
};

struct RuntimeConfig {
  // Sensor trip distances (mm) for the three distance sensors
  double entry_sensor_threshold_mm = 60.0;
  double storage_sensor_threshold_mm = 55.0;
  double top_sensor_threshold_mm = 95.0;

  // Motor voltages (absolute values, direction handled internally)
  double intake_feed_voltage = 10.5;
  double intake_hold_voltage = 1.0;
  double stage2_push_down_voltage = 6.0;
  double stage2_release_voltage = 6.0;
  double outtake_voltage = 11.0;

  // Target ball counts
  int total_ball_target = 11;      // Includes the reserved “stored” ball
  int reserved_storage_slots = 1;  // Number of balls kept in the storage pocket
};

void configureGeometry(const GeometryConfig& config);
void configureRuntime(const RuntimeConfig& config);

// Start or stop the background indexing thread
void enable();              // Alias for setMode(Mode::kIntake)
void disable();             // Alias for setMode(Mode::kDisabled)
void shutdown();            // Stops the thread entirely

void setMode(Mode mode);
Mode getMode();
void startOuttake();        // Shortcut for setMode(Mode::kOuttake)
void stopOuttake();         // Returns to intake mode while staying enabled

// Telemetry / tuning
void resetCounts();
int stackedBallCount();     // Balls visible in the tower (excludes storage pocket)
bool storageBallReady();
int totalBallCount();       // stacked + stored

// Backwards-compatible helpers (adjust runtime config)
void setDetectionThreshold(double threshold_mm);
void setFeedVoltage(double volts);
void setHoldVoltage(double volts);

bool isEnabled();

} // namespace ball_indexer
