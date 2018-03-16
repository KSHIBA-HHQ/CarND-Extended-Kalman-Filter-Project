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

	VectorXd u = VectorXd(4);
	//u << 0, 0, 0, 0;
	//x_=(F_*x_)+u; 	P_=F_*P_*_F.transpose();
	//代わりに誤差行列Qを用いる
	x_=(F_*x_);
	P_=F_*P_*(F_.transpose())+Q_;
		
}

void KalmanFilter::Update(const VectorXd &z) {
	// TODO: update the state by using Kalman Filter equations

	VectorXd y=z-(H_*x_);
	
///////yを適用してUpdate////////////////////////////////////////////////////////////////////
	MatrixXd S=H_*P_*(H_.transpose())+R_;

	MatrixXd K=P_*(H_.transpose())*(S.inverse());
	x_=x_+(K*y);

	long x_size = x_.size();
		
	MatrixXd I=MatrixXd::Identity(x_size, x_size); // Identity matrix(単位行列)

	P_=(I-(K*H_))*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// TODO:update the state by using Extended Kalman Filter equations
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];	

	if(px==0&&py==0)return;
	
	float rho 		= sqrt(px*px + py*py);
	float theta 	= atan2(py , px);
	float rho_dot 	= (px*vx + py*vy) / rho;
	
	VectorXd h = VectorXd(3); // h(x_)
	h << rho, theta, rho_dot;
	
	VectorXd y = z - h;
	
	float PI=atan2(0,-1);
	while(fabs(y[1])>(double)PI){
		if(y[1]>+PI)y[1]-=2.0f*PI;
		if(y[1]<-PI)y[1]+=2.0f*PI;
	}
///////yを適用してUpdate////////////////////////////////////////////////////////////////////
	MatrixXd S=H_*P_*(H_.transpose())+R_;

	MatrixXd K=P_*(H_.transpose())*(S.inverse());
	x_=x_+(K*y);

	long x_size = x_.size();
		
	MatrixXd I=MatrixXd::Identity(x_size, x_size); // Identity matrix(単位行列)

	P_=(I-(K*H_))*P_;	
}
