#include "proj_kalman/kalman_filter.hpp"

int main() 
{
    double t = 1;
    Eigen::MatrixXd Q(4,4);
    Q << 0.1, 0, 0, 0,
        0, 0.1, 0, 0,
        0, 0, 0.1, 0,
        0, 0, 0, 0.1;
    Eigen::MatrixXd R(2,2);
    R << 0.1, 0,
        0, 0.1;

    // without control
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);

    // // with control, acc = 1
    // Eigen::MatrixXd B(4,1);
    // B << 0.5 * pow(t, 2), 0.5 * pow(t, 2), t, t;
    // Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
    // u(0,0) = 1;
    auto kf = std::make_shared<proj_kalman::KalmanFilter>(4, 2, 0, Q, R, B);
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(4);
    kf->init(x_0);
    int iter = 1;
    Eigen::VectorXd z_k = Eigen::VectorXd::Zero(2);
    for (int i = 0; i < iter; i++) {
        Eigen::VectorXd x_p_k = kf->predict(u, t);
        z_k << 1, 1;
        Eigen::VectorXd x_k = kf->update(z_k);
    }

    return 0;
}