#include <cstddef>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

struct data_shared{
    float time;
    float x[4];  // x[0],x[1] for heave, x[2],x[3] for pitch
    float u_base;
    float u_diff_pitch;
    float ref;  // Reference value for heave controller
    float r[2];
    volatile int copy_now;
};

struct throttle_info{
    float vertical_velocity;
    float throttle;
    volatile int copy_now;
};

void init_shm(data_shared* data){
    std::memset(data, 0, sizeof(data_shared));
}

void init_reciever(throttle_info* data){
    std::memset(data, 0, sizeof(throttle_info));
}




//for testing independently this file run this main func

// int main(){
//     Eigen::MatrixXf x(4,1), u(2,1), r(4,1);
//     x << 1,1,1,1;
//     u << 1,2;
//     r << 1,1,1,2;

//     shm_unlink("/states_controller");
//     int fd = shm_open("/states_controller", O_RDWR | O_CREAT, 0666);
//     ftruncate(fd, sizeof(data_shared));

//     data_shared* data = (data_shared*) mmap(NULL, sizeof(data_shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

//     init_shm(data);
//     for (int i = 0 ; i < 100; i++){
//         share_mem(x, u, r, 0.1*i, data);
//         std::cout << i << std::endl;
//     }
//     shm_unlink("/states_controller");
//     close(fd);
//     munmap(data, sizeof(data_shared));
// }
