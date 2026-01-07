#include <vex.h>
#include <iostream>
#include <deque>
#include "definitions.h"
#define CAPACITY 10

struct Ball{
    int id;
    vex::color color;
};

class BallIndexer{
  public:
    BallIndexer(int capacity, bool StoreType) {this->capacity=capacity; this->StoreType=StoreType;}

    void addBall(const Ball& ball){
        Ball temp;
        temp.color=ball.color;
        if (balls.size() == 0 && StoreType) { //First of Long Goal, store in middle goal bit
            temp.id = capacity;
        } else if((balls.size() == 0 && !StoreType) || (StoreType || balls.size() == 2)) { //First of Mid Goal or Second of Long Goal, store in front
            temp.id = 1;
        } else{ //everything past that is the same
            temp.id = balls.back().id + 1;
        }

        balls.push_back(temp);
        return;
    }
    void removeBall(int Type){
        if(Type==1){ //Long Goal
            if(balls.size() > 1){
                Ball temp=balls.front();
                balls.pop_front();
                balls.pop_front();
                balls.push_front(temp);
                for(auto it : balls){
                    it.id--;
                }
            }
            else if(balls.size() == 1){
                balls.pop_front();
            }
        }
        else if(Type==2){
            balls.pop_front();
            for(auto it : balls){
                it.id--;
            }
        }
        else if(Type==3){
            balls.pop_back();
        }
        return;
    }
    int getBallCount() const {return balls.size();}
    void resetIndexer() {capacity=0; balls.clear();};
    Ball getBallAtPosition(int position) const {
        if(position < 0 || position >= balls.size()){
            std::cout<<"Error: Ball position out of range\n";
        }
        return balls[position];
    }

  private:
    int capacity;
    bool StoreType; //1 for long, 0 for mid
    std::deque<Ball> balls;
};

BallIndexer LongGoal(CAPACITY,1), MidGoal(CAPACITY,0);