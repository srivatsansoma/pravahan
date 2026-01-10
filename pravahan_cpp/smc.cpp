#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>


class Discrete_SMC{
public:
    Eigen::MatrixXf f, g, C_t;

    //1-alpha*t should be greater than 0 and less than 1
    float alpha, beta, boundry;

    float time_step;
    Discrete_SMC(
        Eigen::MatrixXf f_,
        Eigen::MatrixXf g_,
        Eigen::MatrixXf C_t_,
        float alpha_,
        float beta_,
        float boundry_,
        float time_step_
    ): f(f_), g(g_), C_t(C_t_), alpha(alpha_), beta(beta_), boundry(boundry_), time_step(time_step_){}
    //when passing the refernce vector should always be one step ahead of the simualtion.
    //the state matrix will keep up with the simulation
    Eigen::MatrixXf update(Eigen::MatrixXf state, Eigen::MatrixXf reference, Eigen::MatrixXf next_reference){
        Eigen::MatrixXf s = C_t*(state - reference);
        Eigen::MatrixXf u = (C_t*g).colPivHouseholderQr().solve(
            (1-alpha*time_step)*s
            - beta*time_step*((s.array()/boundry).tanh()).matrix()
            - C_t*f*state
            + C_t*next_reference
        );

        //debug
        //std::cout<<s<<"\n";

        return u;
    }
};
