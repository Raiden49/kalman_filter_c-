#ifndef PROJ_KALMAN_INTERFACE_HPP_
#define PROJ_KALMAN_INTERFACE_HPP_

#include <cmath>
#include <string>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

namespace proj_kalman {
class KalmanInterface {
    public:
		/**
		 * @brief Construction of Kalman filter interface class
		 * 
		 * @param dim_x dimension of state(x_k)
		 * @param dim_z dimension of measurement(z_k)
		 * @param dim_u dimension of control(u_k)
		 */
        KalmanInterface(int dim_x, int dim_z, int dim_u): 
            dim_x(dim_x), dim_z(dim_z), dim_u(dim_u) {
        x_p_k = Eigen::VectorXd::Zero(dim_x, 1);
        x_l_k = Eigen::VectorXd::Zero(dim_x, 1);
        x_k = Eigen::VectorXd::Zero(dim_x, 1);
        z_k = Eigen::VectorXd::Zero(dim_z, 1);
        A = Eigen::MatrixXd::Identity(dim_x, dim_x);
		K = Eigen::MatrixXd::Zero(dim_x, dim_z);
		Q = Eigen::MatrixXd::Zero(dim_x, dim_x);
		P = Eigen::MatrixXd::Zero(dim_x, dim_x);
		R = Eigen::MatrixXd::Zero(dim_z, dim_z);
    }
	    /**
		* @brief initialization of kalman filter
		*
		* @param x_k initialization of state
		*/
        virtual void init(Eigen::VectorXd &x_k) = 0;

        /**
		* @brief prediction of kalman filter
		*
		* @param u controlling quantity
        * @param t step size
		*/
        virtual Eigen::VectorXd predict(Eigen::VectorXd &u, double t) = 0;

        /**
		* @brief update of kalman filter
		*
		* @param z_k measurement quantity
		*/        
        virtual Eigen::VectorXd update(Eigen::VectorXd &z_k) = 0;

    public:

		int dim_x, dim_z, dim_u;			

		Eigen::VectorXd x_p_k;  // predicted state quantity
		Eigen::VectorXd	x_k;    // updated state quantity
		Eigen::VectorXd	x_l_k;  // last moment state
		Eigen::VectorXd	z_k;    // measurement quantity
		Eigen::VectorXd	u;  // controlling quantity

		Eigen::MatrixXd A;	// state-transition matrix
		Eigen::MatrixXd	P;  // state error covariance matrix
		Eigen::MatrixXd	K;  // gain matrix
		Eigen::MatrixXd	Q;  // process error covariance matrix
		Eigen::MatrixXd R;  // measurement error covariance matrix
    };
}

#endif // PROJ_KALMAN_INTERFACE_HPP_