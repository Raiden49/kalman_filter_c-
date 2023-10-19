#ifndef PROJ_EXTEND_KALMAN_HPP_
#define PROJ_EXTEND_KALMAN_HPP_

#include "proj_kalman/kalman_interface.hpp"

namespace proj_kalman {

const double DEC = M_PI / 180;

class ExtendKalman : public KalmanInterface {
    public:

	/**
	* @brief Construction of Extend Kalman filter class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
	*/
        ExtendKalman(int dim_x, int dim_z, int dim_u) 
            : KalmanInterface(dim_x, dim_z, dim_u) {
            H = Eigen::MatrixXd::Zero(dim_z, dim_x);
        }

        /**
	* @brief Construction of Extend Kalman filter interface class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
        * @param Q process error covariance matrix
        * @param R measurement error covariance matrix
	*/
        ExtendKalman(int dim_x, int dim_z, int dim_u, const Eigen::MatrixXd Q, 
            const Eigen::MatrixXd R) : ExtendKalman(dim_x, dim_z, dim_u) {
            this->Q = Q;
            this->R = R;
        }

	/**
	* @brief initialization of extend kalman filter(override)
	*
	* @param x_k initialization of state
	*/
        void init(Eigen::VectorXd &x_k) override;

        /**
	* @brief prediction of extend kalman filter(override)
	*
	* @param u controlling quantity
        * @param t step size
	*/
        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;

        /**
	* @brief update of extend kalman filter(override)
	*
	* @param z_k measurement quantity
	*/      
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

        // The following functions are for reference only
        Eigen::MatrixXd ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        Eigen::MatrixXd df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        Eigen::MatrixXd se_df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        Eigen::MatrixXd ctrv_sensor(Eigen::VectorXd &x_p_k);
        Eigen::MatrixXd df_ctrv_sensor(Eigen::VectorXd &x_p_k);
        Eigen::MatrixXd se_df_ctrv_sensor(Eigen::VectorXd &x_p_k);

    public:
        Eigen::MatrixXd B;  //  control quantity transfer matrix
        Eigen::MatrixXd H;  //  dimensional transformation matrix
};
}

#endif // PROJ_EXTEND_KALMAN_HPP_
