#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
///	TODO: Calculate the RMSE here.

	int eSize=estimations.size(),gSize=ground_truth.size();
	VectorXd rmse(gSize);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if((eSize != gSize )|| eSize == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}



	//accumulate squared residuals
	for(unsigned int i=0; i < eSize; ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/eSize;

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
///	TODO: Calculate a Jacobian here.
///
///	入力は					x[4]={px,py,vx,vy}　⇒　{x,y,x',y'}の4x1ベクトル
///	出力Radar Measurementsは	Z[3]={range, bearing , range rate}　⇒　{ρ,φ,ρ'}の3x1ベクトル
///
///	したがって求めるヤコビアンは3x4行列（Udacity-lessonと同じ構成）

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	
	//compute the Jacobian matrix
////////////////////////////

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

////////////////////////////
	return Hj;

}
