#ifndef PROJ_KALMAN_FILTER_HPP_
#define PROJ_KALMAN_FILTER_HPP_

#include "proj_kalman/kalman_interface.hpp"

namespace proj_kalman {
class KalmanFilter : public KalmanInterface {
    public:

	/**
	* @brief Construction of Kalman filter class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
	*/
        KalmanFilter(int dim_x, int dim_z, int dim_u) 
            : KalmanInterface(dim_x, dim_z, dim_u) {
            if (dim_u > 0) 
                B = Eigen::MatrixXd::Zero(dim_x, dim_u);
            else
                B = Eigen::MatrixXd::Zero(dim_x, dim_x);
            	H = Eigen::MatrixXd::Zero(dim_z, dim_x);
        }

        /**
	* @brief Construction of Kalman filter interface class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
        * @param Q process error covariance matrix
        * @param R measurement error covariance matrix
	*/
        KalmanFilter(int dim_x, int dim_z, int dim_u, const Eigen::MatrixXd Q, 
            const Eigen::MatrixXd R, const Eigen::MatrixXd B) : KalmanFilter(dim_x, dim_z, dim_u) {
            this->Q = Q;
            this->R = R;
            this->B = B;
        }

	/**
	* @brief initialization of kalman filter(override)
	*
	* @param x_k initialization of state
	*/
        void init(Eigen::VectorXd &x_k) override;

        /**
	* @brief prediction of kalman filter(override)
	*
	* @param u controlling quantity
        * @param t step size
	*/
        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;

        /**
	* @brief update of kalman filter(override)
	*
	* @param z_k measurement quantity
	*/      
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

    public:
        Eigen::MatrixXd B;  //  control quantity transfer matrix
        Eigen::MatrixXd H;  //  dimensional transformation matrix
};
}

#endif // PROJ_KALMAN_FILTER_HPP_
