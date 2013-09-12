#include "statescoremachine.h"
#include <stdio.h>
#include <iostream>


using namespace std;

StateScoreMachine::StateScoreMachine(float power_threshold)
{
    alpha_1 = power_threshold;
}
StateScoreMachine::~StateScoreMachine()
{

}

void StateScoreMachine::updateTrigger()
/*********************************************************
 *This method calculates the real trigger value, given that the
 *hand is over the last target and that the eeg has been activated
 ************************************************************/
{
    cout<<"Trigger is: "<<trigger<<endl;
    cout<<"Final Trigger is: "<<trigger_final<<endl;
    if ((trigger == 1))
    {
        trigger_final = 1;
    }
    if (trigger != 1)
    {
        trigger_final =0;
    }

}

int StateScoreMachine::getTrigger()
{
    return trigger_final;
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

int StateScoreMachine::getFirstTrigger()
{
    return first_trigger;
}

void StateScoreMachine::reset_states()
{
    first_trigger =0;
    rest_counter=0;
    active_counter=0;
    trigger = 0;
    target=0;
}

void StateScoreMachine::CalculateChange(float mean_power)
{
    cout<<"Active Counter is:"<<active_counter<<endl;
    cout<<"Rest COunter is:"<<rest_counter<<endl;
    cout<<"Trigger is: "<<trigger<<endl;
    cout<<"Target is: "<<target<<endl;

    if (mean_power>alpha_1){
        rest_counter+=1;
      }
    else{
        rest_counter=0;

    }
    if(target==0){//This is the rest state
        if(rest_counter>=10){
            target=1;
            first_trigger = 1;
            reward+=0.25;
        }
    }
    else{ //This is the active state
        if ((active_counter<200)&&(trigger!=1)) { //it was 150
            target=1;
            active_counter+=1;
        }
        else{
        target=0;//Change the cue to zero
        trigger = 0; //Restart the trigger state
        first_trigger=0;
        rest_counter=0;
        active_counter=0;
        }

    }
    if(target==1){
        //
        if(mean_power<power_threshold){
            if(active_counter>=10){
                trigger = 1; //fire the trigger
                active_counter = 200; //Run till the end of the counter
            }
            if(active_counter==30){
                reward+=0.5;
            }
        }
    }
}
