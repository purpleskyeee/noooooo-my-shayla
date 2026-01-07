#ifndef BALL_INDEXER_H
#define BALL_INDEXER_H
#include "vex.h"
#include <deque>

struct Ball {
    int id;
    vex::color color;
};

class BallIndexer
{
    public:
    BallIndexer(int capacity, bool StoreType);
    void addBall(const Ball& ball);
    void removeBall(int Type);
    int getBallCount() const;
    void resetIndexer();
    Ball getBallAtPosition(int position) const;

  private:
    int capacity;
    bool StoreType; //1 for long, 0 for mid
    std::deque<Ball> balls;
};

extern BallIndexer LongGoal;
extern BallIndexer MidGoal;

#endif // BALL_INDEXER_H