#ifndef STATESCOREMACHINE_H
#define STATESCOREMACHINE_H
using namespace std;

class StateScoreMachine
{
private:
    float reward=0.0;
    float target;
    float alpha_1;
    float power_threshold;
    int price_counter = 0;
    int rest_counter=0;
    int active_counter=0;
    int trigger = 0;
    int trigger_final = 0;
    int first_trigger =0;

public:
    StateScoreMachine(float power_threshold);
    ~StateScoreMachine();
    float getReward();
    float getState();
    int getTrigger();
    int getFirstTrigger();
    void setThreshold(float threshold_power);
    void CalculateChange(float mean_power);
    void reset_states();
    void updateTrigger();
};

#endif // STATESCOREMACHINE_H
