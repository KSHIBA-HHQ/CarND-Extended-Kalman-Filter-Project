#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Constructor.
FusionEKF::FusionEKF() {
  reset_item();
}

void FusionEKF::reset_item(){  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 	0.0225, 0.0000,
				0.0000, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 	0.09, 0.0000, 0.00,
				0.00, 0.0009, 0.00,
				0.00, 0.0000, 0.09;

  
  //TODO:
  // >>Finish initializing the FusionEKF.
  // >>Set the process and measurement noises
  
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

}


// Destructor.
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    
    //TODO:
    //  >> Initialize the state ekf_.x_ with the first measurement.
    //  >> Create the covariance matrix.
    //  >> Remember: you'll need to convert radar from polar to cartesian coordinates.
   
	
	MatrixXd P (4, 4);
    P  <<  1, 0, 0,    0,
		   0, 1, 0,    0,
		   0, 0, 1000, 0,
		   0, 0, 0, 1000;
			   
			   
    // first measurement
    cout << "EKF: " << endl;
	
	VectorXd x(4);
	MatrixXd H,R;
			
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
		//Convert radar from polar to cartesian coordinates and initialize state.
		float rho = measurement_pack.raw_measurements_[0]; 		// range
		float phi = measurement_pack.raw_measurements_[1]; 		// bearing
		float rho_dot = measurement_pack.raw_measurements_[2]; 	// velocity of rho
		// Coordinates convertion from polar to cartesian
		float px = rho * cos(phi); 
		float py = rho * sin(phi);
		float vx = rho_dot * cos(phi);
		float vy = rho_dot * sin(phi);
		x << px, py, vx , vy; 

		H=Hj_ = tools.CalculateJacobian(ekf_.x_);
		R=R_radar_;	

	}
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
		//Initialize state.
		x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		H=H_laser_;
		R=R_laser_;


	}	   
   
	MatrixXd F(4, 4);
	F << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	MatrixXd Q(4,4);
	Q << 	0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;


	ekf_.Init(	x, 	/*x_in*/ 
				P, 	/*P_in*/ 
				F, 	/*F_in*/
				H, 	/*H_in*/ 
				R, 	/*R_in*/  
				Q );/*Q_in*/

	
    // rec initial timestamp for next dt calculation
    previous_timestamp_ = measurement_pack.timestamp_;
	
	
    // done initializing, no need to predict or update	
		
    is_initialized_ = true;
    return;
  }

  // ****************************************************************************
  // Prediction
  // ****************************************************************************

  
	// TODO:
	//  >> Update the state transition matrix F according to the new elapsed time.
	//    - Time is measured in seconds.
	//  >> Update the process noise covariance matrix.

	//[lesson 8 :State Prediction]
	float dt = (measurement_pack.timestamp_ - previous_timestamp_);
	dt /= 1000000.0; // convert micros to s
	previous_timestamp_ = measurement_pack.timestamp_;

	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 	1, 0, dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1;
	
	
	//[lesson 9 :Process Covariance Matrix]
	//  >> Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	float _ax =9;	//noise_ax
    float _ay =9;	//noise_ay
  
	float dt_2 = dt * dt; //dt^2
	float dt_3 = dt_2 * dt; //dt^3
	float dt_4 = dt_3 * dt; //dt^4
	float dt_4p4 = dt_4 / 4; //dt^4/4
	float dt_3p2 = dt_3 / 2; //dt^3/2
	
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<	 dt_4p4*_ax,          0, dt_3p2*_ax,          0,
				          0, dt_4p4*_ay,          0, dt_3p2*_ay,
				 dt_3p2*_ax,          0,   dt_2*_ax,          0,
				          0, dt_3p2*_ay,          0,   dt_2*_ay;

	
	ekf_.Predict();

  // ****************************************************************************
  // Update
  // ****************************************************************************

  
  // TODO:
  //   >> Use the sensor type to perform the update step.
  //   >> Update the state and covariance matrices.
  
//MatrixXd H_ 	// measurement matrix
//MatrixXd R_	// measurement covariance matrix
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	// Radar updates
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	
  } else {
	// Laser updates
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
