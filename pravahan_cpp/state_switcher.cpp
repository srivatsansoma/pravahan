#include <cmath>
#define GRAVITY 9.81

float velocity;
float acc;

float max_theta = 6;
float min_theta = -6;

float I = 1;
float m = 1;
float L1,L2 = 10;
float d1,d2 = 0.3;

float ratio_L1d1_2L2d2 = L1*d1/2/L2/d2;

float alpha_front_eq = 0;
float alpha_back_eq = 0;

float takeoff_initial_h = 0.5;
float takeoff_factor = 1.1;

void update_eq_alphas_after_takeoff(float &alpha_f, float &alpha_b){
    alpha_f = m*GRAVITY/(L1 + 2*L2/ratio_L1d1_2L2d2)/velocity/velocity;
    alpha_b = alpha_f/ratio_L1d1_2L2d2;
}

float min_vel_takeoff = std::sqrt(m*GRAVITY/(L1*max_theta + 2*L2*max_theta/ratio_L1d1_2L2d2));

void update_stage1(float &vel){
    alpha_front_eq = vel/velocity * max_theta;
    alpha_back_eq = alpha_front_eq/ratio_L1d1_2L2d2;
}

float update_takeoff_stage(){
    update_eq_alphas_after_takeoff(alpha_front_eq, alpha_back_eq);
    return takeoff_initial_h*(velocity - min_vel_takeoff)/(takeoff_factor-1)/min_vel_takeoff;
}
