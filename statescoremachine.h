#ifndef STATESCOREMACHINE_H
#define STATESCOREMACHINE_H

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

public:
    StateScoreMachine(float power_threshold);
    ~StateScoreMachine();
    float getReward();
    float getState();
    void setThreshold(float threshold_power);
    void CalculateChange(float mean_power);
};

#endif // STATESCOREMACHINE_H
