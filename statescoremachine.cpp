#include "statescoremachine.h"

StateScoreMachine::StateScoreMachine(float power_threshold)
{
    alpha_1 = power_threshold;
}
StateScoreMachine::~StateScoreMachine()
{

}

float StateScoreMachine::getReward()
{
    return reward;
}

float StateScoreMachine::getState()
{
    return target;
}

void StateScoreMachine::setThreshold(float threshold_power)
{
    power_threshold = threshold_power;
}

void StateScoreMachine::CalculateChange(float mean_power)
{
    if (mean_power>alpha_1){
        rest_counter+=1;
      }
    else{
        rest_counter=0;

    }
    if(target==0){//This is the rest state
        if(rest_counter>=1*20){
            target=1;
            reward+=0.25;
        }
    }
    else{ //This is the active state
        if (active_counter<150) {
            target=1;
            active_counter+=1;
        }
        else{
        target=0;//Change the cue to zero
        rest_counter=0;
        active_counter=0;
        }

    }
    if(target==1){
        //
        if(mean_power<power_threshold){
            if(active_counter==75){
                reward+=0.5;
            }
        }
    }
}
