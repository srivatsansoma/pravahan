#include <vector>

class PID{
public:
    float prev_error = 0;
    float integral = 0;

    float kp = 0;
    float kd = 0;
    float ki = 0;

    PID(std::vector<float> &state_vec_, std::vector<float> &ref_, float kp_, float kd_, float ki_) :
        kp(kp_), kd(kd_), ki(ki_) {}

    float update(float x, float ref,float time_step){
        float error = ref - x;
        integral += error*time_step;
        float diff = (error - prev_error)/time_step;
        prev_error = error;

        return (kp*error + kd*diff + ki*integral);
    }
};
