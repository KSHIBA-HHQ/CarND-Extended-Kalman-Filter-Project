#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
	// TODO:predict the state

	VectorXd u = VectorXd(2);
	u << 0, 0;
	x_=(F_*x_)+u;

	//P=F*P*F.transpose();
	P_=F_*P_*F_.transpose()+Q_;
		
}

void KalmanFilter::Update(const VectorXd &z) {
	// TODO: update the state by using Kalman Filter equations

	VectorXd y=z-(H_*x_);
	
///////yを適用してUpdate////////////////////////////////////////////////////////////////////
	MatrixXd S=H_*P_*H_.transpose()+R_;

	MatrixXd K=P_*H_.transpose()*S.inverse();
	x_=x_+(K*y);

	long x_size = x_.size();
		
	MatrixXd I=MatrixXd::Identity(x_size, x_size); // Identity matrix(単位行列)

	P_=(I-(K*H_))*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// TODO:update the state by using Extended Kalman Filter equations
	double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	double theta = atan(x_(1) / x_(0));
	double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
	
	VectorXd h = VectorXd(3); // h(x_)
	h << rho, theta, rho_dot;
	
	VectorXd y = z - h;
	
///////yを適用してUpdate////////////////////////////////////////////////////////////////////
	MatrixXd S=H_*P_*H_.transpose()+R_;

	MatrixXd K=P_*H_.transpose()*S.inverse();
	x_=x_+(K*y);

	long x_size = x_.size();
		
	MatrixXd I=MatrixXd::Identity(x_size, x_size); // Identity matrix(単位行列)

	P_=(I-(K*H_))*P_;	
}
