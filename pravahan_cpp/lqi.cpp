#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>
#include <csignal>

#include "memory_sharing.cpp"
#include "state_space.cpp"

#define GRAVITY 9.81

float dt = 0.1;

//in this func you should give in discretized values.
Eigen::Matrix<float, 1,3> solve_gain(
    Eigen::Matrix<float, 2,2> A,
    Eigen::Matrix<float, 2,1> B,
    Eigen::Matrix<float, 1,2> C,
    Eigen::Matrix<float, 1,1> D,
    Eigen::Matrix<float, 3,3> Qa,
    Eigen::Matrix<float, 1,1> R
){
    Eigen::Matrix<float, 3,3> Aa = Eigen::Matrix<float, 3, 3>::Zero();
    Aa.block<2,2>(0,0) = A;
    Aa.block<2,1>(0,2) = Eigen::Matrix<float, 2,1>::Zero();
    Aa.block<1,2>(2,0) = -C;
    Aa(2,2) = 1;

    Eigen::Matrix<float,3,1> Ba = Eigen::Matrix<float,3,1>::Zero();
    Ba.block<2,1>(0,0) = B;

    Eigen::Matrix<float, 3,3> P = Qa;
    for (int i = 0 ; i < 100; i++){
        P = Aa.transpose()*P*Aa
            - Aa.transpose()*P*Ba*(R + Ba.transpose()*P*Ba).colPivHouseholderQr().solve(Ba.transpose()*P*Aa)
            + Qa;
    }

    Eigen::Matrix<float, 1,3> K = (R + Ba.transpose()*P*Ba).colPivHouseholderQr().solve(Ba.transpose()*P*Aa);

    return K;
}

float L = 1.4;
float d1 = 0.3, d2 = 0.3;
float I = 5, M = 5;

std::atomic<bool> run_program(true);
bool run_controller_heave = true;
bool run_controller_pitch = true;

// Global pointers for cleanup
throttle_info* g_throttle_controls_pitch = nullptr;
throttle_info* g_throttle_controls_heave = nullptr;
data_shared* g_data_to_send_pitch = nullptr;
data_shared* g_data_to_send_heave = nullptr;
int g_fd_receiver_pitch = -1;
int g_fd_receiver_heave = -1;
int g_fd_send_pitch = -1;
int g_fd_send_heave = -1;

void cleanup_resources() {
    std::cout << "\nCleaning up resources...\n";
    
    // Cleanup pitch controller resources
    if (g_throttle_controls_pitch != nullptr) {
        munmap(g_throttle_controls_pitch, sizeof(throttle_info));
        g_throttle_controls_pitch = nullptr;
    }
    if (g_fd_receiver_pitch >= 0) {
        close(g_fd_receiver_pitch);
        g_fd_receiver_pitch = -1;
    }
    if (g_data_to_send_pitch != nullptr) {
        munmap(g_data_to_send_pitch, sizeof(data_shared));
        //std::cout << "debug_info : send_pitch is null ptr";
        g_data_to_send_pitch = nullptr;
    }
    if (g_fd_send_pitch >= 0) {
        close(g_fd_send_pitch);
        //std::cout << "debug_info : send_pitch file descriptor not opening";
        g_fd_send_pitch = -1;
    }
    
    // Cleanup heave controller resources
    if (g_throttle_controls_heave != nullptr) {
        munmap(g_throttle_controls_heave, sizeof(throttle_info));
        g_throttle_controls_heave = nullptr;
    }
    if (g_fd_receiver_heave >= 0) {
        close(g_fd_receiver_heave);
        g_fd_receiver_heave = -1;
    }
    if (g_data_to_send_heave != nullptr) {
        munmap(g_data_to_send_heave, sizeof(data_shared));
        g_data_to_send_heave = nullptr;
    }
    if (g_fd_send_heave >= 0) {
        close(g_fd_send_heave);
        g_fd_send_heave = -1;
    }
    
    // Unlink shared memory (only need to do once)
    shm_unlink("/controls");
    shm_unlink("/cpp_to_py");
    
    std::cout << "Cleanup complete.\n";
}

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived interrupt signal. Shutting down...\n";
        run_program = false;
        cleanup_resources();
        exit(0);
    }
}

float velocity = 2;
float max_angle_possible = 10;
float min_velocity_for_takeoff = sqrt(M*GRAVITY/(L + 4*L*d2/d1)/max_angle_possible);

std::atomic<int> controllers_completed(0);

float max_velocity_for_takeoff = M*GRAVITY/(max_angle_possible)

std::vector<Eigen::Matrix<float, 1,3>> init_compute_pitch(float start_speed, float end_speed, float step_size){
    std::vector<Eigen::Matrix<float, 1,3>> gains = {};
    float i = start_speed;

    Eigen::Matrix<float,2,2> A = Eigen::Matrix<float, 2,2>::Zero();

    Eigen::Matrix<float,2,2> Ad = (A*dt).exp();

    Eigen::Matrix<float, 1, 2> C = Eigen::Matrix<float, 1,2>();
    C<<1,0;
    Eigen::Matrix<float, 1, 2> Cd = C*dt;
    Eigen::Matrix<float, 1, 1> D = Eigen::Matrix<float, 1,1>::Zero();

    Eigen::Matrix<float,3,3> Q = Eigen::Matrix<float,3,3>::Zero();
    Q.diagonal() << 4000, 100, 10;

    Eigen::Matrix<float, 1,1> R;
    R << 10;
    while(i < end_speed){
        float k2 = (-L*d1 + 4*L*d2) * i * i;
        A << 0, 1,
             -k2, -0.3;
        Eigen::Matrix<float,2,2> Ad = (A*dt).exp();

        Eigen::Matrix<float, 2,1> B;
        B << 0,
             -k2;
        Eigen::Matrix<float, 2,1> Bd = A.inverse() * (Ad - Eigen::Matrix2f::Identity()) * B;

        Eigen::Matrix<float,1,3> K = solve_gain(Ad, Bd, Cd, D, Q, R);
        gains.push_back(K);
        //std::cout << K;

        i += step_size;
    }

    return gains;
}

std::vector<Eigen::Matrix<float, 1,3>> init_compute_heave(float start_speed, float end_speed, float step_size) {
    std::vector<Eigen::Matrix<float, 1,3>> gains = {};
    float i = start_speed;

    Eigen::Matrix<float,2,2> A = Eigen::Matrix<float, 2,2>::Zero();

    Eigen::Matrix<float,2,2> Ad = (A*dt).exp();

    Eigen::Matrix<float, 1, 2> C = Eigen::Matrix<float, 1,2>();
    C<<1,0;
    Eigen::Matrix<float, 1, 2> Cd = C*dt;
    Eigen::Matrix<float, 1, 1> D = Eigen::Matrix<float, 1,1>::Zero();

    Eigen::Matrix<float,3,3> Q = Eigen::Matrix<float,3,3>::Zero();
    Q.diagonal() << 4000, 100, 10;

    Eigen::Matrix<float, 1,1> R;
    R << 10;
    while(i < end_speed){
        float k1 = (L + 4*L*d2/d1) * i * i;
        A << 0, 1,
             -k1, -0.3;
        Eigen::Matrix<float,2,2> Ad = (A*dt).exp();

        Eigen::Matrix<float, 2,1> B;
        B << 0,
             -k1;
        Eigen::Matrix<float, 2,1> Bd = A.inverse() * (Ad - Eigen::Matrix2f::Identity()) * B;

        Eigen::Matrix<float,1,3> K = solve_gain(Ad, Bd, Cd, D, Q, R);
        gains.push_back(K);
        //std::cout << K;

        i += step_size;
    }

    return gains;
}

//common to both pitch and heave controllers
Eigen::Matrix<float, 1, 3> compute_gain(float vel,float start, float step_size, std::vector<Eigen::Matrix<float, 1,3>>& gains) {
    // Clamp velocity to valid range
    float end = start + step_size * (gains.size() - 1);
    if (vel < start) vel = start;
    if (vel >= end) vel = end - 0.001f; // Just below end to avoid out of bounds

    int base = std::floor((vel - start)/step_size);
    // Ensure base+1 is within bounds
    if (base + 1 >= gains.size()) base = gains.size() - 2;

    Eigen::Matrix<float, 1, 3> result = (vel - (start + step_size*base))/step_size * gains.at(base)
                                        + ((start + step_size*(base+1)) - vel)/step_size * gains.at(base+1);

    return result;
}

float acc_factor = 3;

float GLOBAL_TIME = 0;
float GLOBAL_TICK = 0;

void timer(int time_period_ms){
    while(run_program) {
        std::this_thread::sleep_for(std::chrono::milliseconds(time_period_ms));
        GLOBAL_TIME += (float)time_period_ms/1000;
        GLOBAL_TICK++;
        if (run_controller_heave || run_controller_pitch) {
            std::cout << "timer too fast" << "\n";
        }
        run_controller_heave = true;
        run_controller_pitch = true;
    }
}

void controller_pitch(){
    std::vector<Eigen::Matrix<float, 1, 3>> gains = init_compute_pitch(0, 100, 0.5);
    std::cout << "init done for controller gains.."  << "\n";

    float k2 =( -L*d1 + 4*L*d2) * velocity * velocity;

    Eigen::Matrix<float,2,2> A = Eigen::Matrix<float, 2,2>::Zero();
    A << 0, 1,
         -k2, -0.3;

    Eigen::Matrix<float, 2,1> B;
    B << 0,
        -k2;

    Eigen::Matrix<float, 1, 2> C = Eigen::Matrix<float, 1,2>();
    C<<1,0;
    Eigen::Matrix<float, 1, 1> D = Eigen::Matrix<float, 1,1>::Zero();

    Eigen::Matrix<float, 2, 1> init_state;
    init_state << 5,
                  0;
    variable_ss_discrete dss(A, B, C, D, init_state, dt);

    Eigen::Matrix<float, 1, 1> integral_term = Eigen::Matrix<float, 1, 1>::Zero();

    shm_unlink("/controls");
    int fd_reciever = shm_open("/controls", O_RDWR | O_CREAT, 0666);
    ftruncate(fd_reciever, sizeof(throttle_info));
    throttle_info* throttle_controls = (throttle_info*) mmap(NULL, sizeof(throttle_info), PROT_READ | PROT_WRITE, MAP_SHARED, fd_reciever, 0);
    
    // Store for cleanup
    g_throttle_controls_pitch = throttle_controls;
    g_fd_receiver_pitch = fd_reciever;

    int fd_send = shm_open("/cpp_to_py", O_RDWR | O_CREAT, 0666);
    ftruncate(fd_send, sizeof(data_shared));
    data_shared* data_to_send = (data_shared*) mmap(NULL, sizeof(data_shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd_send, 0);
    
    // Store for cleanup
    g_data_to_send_pitch = data_to_send;
    g_fd_send_pitch = fd_send;
    
    // Initialize shared memory to zero (only first controller to create it)
    if (data_to_send->copy_now == 0 && controllers_completed == 0) {
        init_shm(data_to_send);
    }

    while(run_program) {
        if (run_controller_pitch){
            if (throttle_controls->copy_now){
                velocity += throttle_controls->throttle * acc_factor * dt;
                std::cout << velocity << "\n";
                //std::cout << throttle_controls->vertical_velocity << " " << throttle_controls->throttle << std::endl;
                throttle_controls->copy_now = 0;
            }

            Eigen::Matrix<float, 1, 1> u;

            // Low-speed handling for pitch controller
            if (velocity <= min_velocity_for_takeoff){
                // At low speeds, use simple proportional control to stabilize
                u(0,0) = -0.1 * dss.x(0,0) - 0.05 * dss.x(1,0); // Simple PD control
                dss.update(u);
            }
            else{
                // Compute k2 FIRST, then update A and B
                k2 = (-L*d1 + 4*L*d2) * velocity * velocity;
                A << 0, 1,
                     -k2, -0.3;
                B << 0,
                    -k2;

                // Update discrete system matrices when k2 changes
                dss.updateAB(A, B);

                Eigen::Matrix<float, 1,3> gain = compute_gain(velocity, 0, 0.5, gains);
                u = -gain.block<1,2>(0,0)*dss.x - gain.block<1,1>(0,2)*integral_term;
                //std::cout << dss.y << "\n";
                dss.update(u);
                integral_term -= dss.y*dt;
            }

            if (data_to_send->copy_now == 0 && controllers_completed < 2){
                memcpy(data_to_send->x + 2, dss.x.data(), 2*sizeof(float));
                data_to_send->u_diff_pitch = u(0,0);
                data_to_send->time = GLOBAL_TIME;
                controllers_completed ++;
            }
            if (data_to_send->copy_now == 0 && controllers_completed == 2){
                data_to_send->copy_now = 1;
                controllers_completed = 0;
            }

            run_controller_pitch = false;
        }
    }

    // Cleanup is handled by signal handler
    // Resources are stored globally for cleanup on Ctrl+C
}

void controller_heave(){
    std::vector<Eigen::Matrix<float, 1, 3>> gains = init_compute_heave(0, 100, 0.5);
    std::cout << "init done for controller gains.."  << "\n";

    float k1 = (L + 4*L*d2/d1) * velocity * velocity;

    Eigen::Matrix<float,2,2> A = Eigen::Matrix<float, 2,2>::Zero();
    A << 0, 1,
         -k1, -0.3;

    Eigen::Matrix<float, 2,1> B;
    B << 0,
        -k1;

    Eigen::Matrix<float, 1, 2> C = Eigen::Matrix<float, 1,2>();
    C<<1,0;
    Eigen::Matrix<float, 1, 1> D = Eigen::Matrix<float, 1,1>::Zero();

    Eigen::Matrix<float, 2, 1> init_state;
    init_state << 0,
                  0;

    Eigen::Matrix<float, 1, 1> ref;
    ref << 1;
    variable_ss_discrete dss(A, B, C, D, init_state, dt);

    Eigen::Matrix<float, 1, 1> integral_term = Eigen::Matrix<float, 1, 1>::Zero();

    shm_unlink("/controls");
    int fd_reciever = shm_open("/controls", O_RDWR | O_CREAT, 0666);
    ftruncate(fd_reciever, sizeof(throttle_info));
    throttle_info* throttle_controls = (throttle_info*) mmap(NULL, sizeof(throttle_info), PROT_READ | PROT_WRITE, MAP_SHARED, fd_reciever, 0);
    
    // Store for cleanup
    g_throttle_controls_pitch = throttle_controls;
    g_fd_receiver_pitch = fd_reciever;

    int fd_send = shm_open("/cpp_to_py", O_RDWR | O_CREAT, 0666);
    ftruncate(fd_send, sizeof(data_shared));
    data_shared* data_to_send = (data_shared*) mmap(NULL, sizeof(data_shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd_send, 0);
    
    // Store for cleanup
    g_data_to_send_pitch = data_to_send;
    g_fd_send_pitch = fd_send;
    
    // Initialize shared memory to zero (only first controller to create it)
    if (data_to_send->copy_now == 0 && controllers_completed == 0) {
        init_shm(data_to_send);
    }

    while(run_program) {
        if (run_controller_heave){
            Eigen::Matrix<float, 1, 1> u;
            std::cout << velocity << "\n";
            if (throttle_controls->copy_now){
                velocity += throttle_controls->throttle * acc_factor * dt;
                //std::cout << throttle_controls->vertical_velocity << " " << throttle_controls->throttle << std::endl;
            }
            if (velocity <= min_velocity_for_takeoff){
                u(0,0) = max_angle_possible * velocity / min_velocity_for_takeoff;
                dss.update(u);
            }
            else{
                // Compute k1 FIRST, then update A and B
                k1 = (L + 4*L*d2/d1) * velocity * velocity;
                A << 0, 1,
                     -k1, -0.3;
                B << 0,
                    -k1;

                // Update discrete system matrices when k1 changes
                dss.updateAB(A, B);

                Eigen::Matrix<float, 1,3> gain = compute_gain(velocity, 0, 0.5, gains);
                u = -gain.block<1,2>(0,0)*dss.x - gain.block<1,1>(0,2)*integral_term;
                //std::cout << dss.y << "\n";
                dss.update(u);
                integral_term -= (dss.y-ref)*dt;
            }

            run_controller_heave = false;

            if (data_to_send->copy_now == 0 && controllers_completed < 2){
                memcpy(data_to_send->x, dss.x.data(), 2*sizeof(float));
                data_to_send->u_base = u(0,0);
                data_to_send->ref = ref(0,0);
                data_to_send->time = GLOBAL_TIME;
                controllers_completed ++;
            }
            if (data_to_send->copy_now == 0 && controllers_completed == 2){
                data_to_send->copy_now = 1;
                controllers_completed = 0;
            }

            if (throttle_controls->copy_now){
                ref(0,0) += throttle_controls->vertical_velocity * dt;
                if (ref(0,0) > 3.0f) ref(0,0) = 3.0f;
                throttle_controls->copy_now = 0;

            }
            std::cout << ref(0,0) << " " << dss.y(0,0) << "\n";
        }
    }

    // Cleanup is handled by signal handler
    // Resources are stored globally for cleanup on Ctrl+C
}


int main(){
    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::thread timer_thread(timer, 10);
    std::thread controller_thread_heave(controller_heave);
    std::thread controller_thread_pitch(controller_pitch);

    timer_thread.join();
    controller_thread_heave.join();
    controller_thread_pitch.join();

    // Cleanup on normal exit
    cleanup_resources();

    // std::vector<Eigen::Matrix<float, 1,3>> gains = init_compute(0, 10, 0.5);
    // std::cout << gains[0];
}
