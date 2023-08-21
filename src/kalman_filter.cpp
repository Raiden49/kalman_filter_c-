#include "proj_kalman/kalman_filter.hpp"

#include <iostream>

namespace proj_kalman {
void KalmanFilter::init(Eigen::VectorXd &x_k) {
    this->x_p_k = x_k;
    this->x_l_k = x_k;
    this->P = Eigen::MatrixXd::Zero(dim_x, dim_x);
}

Eigen::VectorXd KalmanFilter::predict(Eigen::VectorXd &u, double t) {
    double delta_t = t;
    for (int i = 0; i < dim_x/2; i++) {
        A(i, dim_x/2 + i) = delta_t;
    }
    x_p_k = A * x_l_k + B * u;
	P = A * P * A.transpose() + Q;
	return x_p_k;
}

Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd &z_k) {
    for (int i = 0; i < dim_z; i++) {
        H(i,i) = 1;
    }
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
	x_k = x_p_k + K * (z_k - H * x_p_k);
	P = P - K * H * P;
    x_l_k = x_k;
	return x_k;
}

}