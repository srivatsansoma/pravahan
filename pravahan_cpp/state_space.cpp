#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

class variable_ss_discrete{
public:
    Eigen::MatrixXf A, B, C, D;

    Eigen::MatrixXf x,y;

    float time_step;

    variable_ss_discrete(
        Eigen::MatrixXf A_,
        Eigen::MatrixXf B_,
        Eigen::MatrixXf C_,
        Eigen::MatrixXf D_,
        Eigen::MatrixXf init_state,
        float time_step_
    ) : C(C_), D(D_), x(init_state), time_step(time_step_) {
        updateAB(A_, B_);
    }

    void updateAB(Eigen::MatrixXf A_, Eigen::MatrixXf B_) {
        int A_size = A_.rows();
        int B_col_size = B_.cols();

        Eigen::MatrixXf van_loan = Eigen::MatrixXf::Zero(A_size+B_col_size, A_size+B_col_size);
        van_loan << A_ , B_,
                    Eigen::MatrixXf::Zero(B_col_size, A_size), Eigen::MatrixXf::Zero(B_col_size, B_col_size);

        van_loan = (van_loan*time_step).exp();

        A = van_loan.block(0,0, A_size, A_size);
        B = van_loan.block(0, A_size, A_size, B_col_size);
    }
    void updateC(Eigen::MatrixXf C_) {C = C_;}
    void updateD(Eigen::MatrixXf D_) {D = D_;}

    void update(Eigen::MatrixXf u){
        x = A*x + B*u;
        y = C*x + D*u;
    }
};
