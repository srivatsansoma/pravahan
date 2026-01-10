/*
    Learnings:
    -in a fast env dont use growing vectors because of synicing issues
     when the vector grows and need to be copied somwhere else in mem
    -dont use Eigen map with memory that will be erased before the matrix itself.
*/

#include <chrono>
#include <iterator>
#include <thread>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "smc.cpp"
#include "state_space.cpp"
#include "memory_sharing.cpp"
#include <cmath>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>

#define GRAVITY 9.81

float velocity = 0.0001;
float heave;

float max_theta = 20;
float min_theta = -6;

float I = 1;
float m = 1;
float L1 = 10, L2 = 10;
float d1 = 0.3,d2 = 0.3;

float ratio_L1d1_2L2d2 = L1*d1/2/L2/d2;

float alpha_front_eq = 0;
float alpha_back_eq = 0;

float max_takeoff_h = 0.5;
float takeoff_factor = 1.1;
float min_takeoff_time = 3;
float min_landing_time = 3;

float acc_factor = 3;

float min_vel_takeoff = std::sqrt(m*GRAVITY/(L1*max_theta + 2*L2*max_theta/ratio_L1d1_2L2d2));

void update_stage1(float &vel){
    alpha_front_eq = vel/velocity * max_theta;
    alpha_back_eq = alpha_front_eq/ratio_L1d1_2L2d2;
}

bool takeoff_mode = false;
bool landing_mode = false;
bool run_program = true;
bool run_controller = true;

float GLOBAL_TIME = 0;
float GLOBAL_TICK = 0;

void timer_1Hz(){
    while(run_program) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        GLOBAL_TIME += 1;
        GLOBAL_TICK++;
        if (run_controller) {
            std::cout << "timer too fast";
            while(1){}
        }
        run_controller = true;
    }
}

void timer_100Hz(){
    while(run_program) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        GLOBAL_TIME += 0.01;
        GLOBAL_TICK++;
        if (run_controller) {
            std::cout << "timer too fast";
            while(1){}
        }
        run_controller = true;
    }
}

void SMC(){
    float dt = 0.01;

    Eigen::MatrixXf f(2,2);
    f << 0,1,-5,-3;

    Eigen::MatrixXf g(2, 1);
    g << 0, 1;

    Eigen::MatrixXf C_t(1,2);
    C_t <<1, 1;

    Eigen::MatrixXf state(2,1);
    state << 0.5, -0.5;

    Eigen::MatrixXf ref(2,1);
    ref << 1,0 ;

    Eigen::MatrixXf next_ref(2,1);
    ref << 1,0 ;

    variable_ss_discrete dss(
        f,
        g,
        Eigen::MatrixXf::Identity(2,2),
        Eigen::MatrixXf::Zero(2,1),
        state,
        dt
    );
    Discrete_SMC dsmc(dss.A, dss.B, C_t, 0, 1, 1, dt);

    shm_unlink("/controls");
    int fd_reciever = shm_open("/controls", O_RDWR | O_CREAT, 0666);
    ftruncate(fd_reciever, sizeof(throttle_info));
    throttle_info* throttle_controls = (throttle_info*) mmap(NULL, sizeof(throttle_info), PROT_READ | PROT_WRITE, MAP_SHARED, fd_reciever, 0);

    Eigen::MatrixXf u(1,1); u << 0;
    while(run_program) {
        float next_ref_;
        if (throttle_controls->copy_now){
            ref  = next_ref;
            next_ref(0,0) += throttle_controls->vertical_velocity * dt;
            if (next_ref(0,0) > 3.0f) next_ref(0,0) = 3.0f;
            velocity += throttle_controls->throttle * acc_factor * dt;
            //std::cout << throttle_controls->vertical_velocity << " " << throttle_controls->throttle << std::endl;
            throttle_controls->copy_now = 0;
        }
        if (run_controller) {
            if (velocity < min_vel_takeoff) {
                update_stage1(velocity);
            }else {
                Eigen::MatrixXf temp_u = dsmc.update(state, ref, next_ref);
                if (std::fabs(temp_u(0,0)) <= max_theta)
                {
                    u = temp_u;
                }

                dss.update(u);

                alpha_front_eq = u(0,0);
                //alpha_back_eq  = u(1,0);

                state = dss.x;

                //debug
                std::cout << "y = " << dss.y << "\n";
                //std::cout << u << "\n";


                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            run_controller = false;
        }
    }
}


int main(){
    std::thread time_10ms(timer_100Hz);
    std::thread controller(SMC);

    time_10ms.join();
    controller.join();
}
