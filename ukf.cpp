#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.628;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */

  // 7 rows: 5 from input x and 2 for augumented data; columns = 2 * augumented + 1 
  Xsig_pred_ = MatrixXd(5,15);

  n_x_ = 5;
  n_aug_ = 7;

  lambda_ = 3 - n_x_;
  
  weights_ = VectorXd(2*n_aug_+1); 
  
  P_ << 1, 0, 0, 0, 0,
	0, 1, 0, 0, 0,
	0, 0, 1, 0, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 1;
 


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  cout << "--------------------------------------------------------------" << endl;
  cout << "--------------Start new iteration-----------------------------" << endl;
  cout << "--------------------------------------------------------------" << endl;

  if (!is_initialized_) {
	
	cout << "initialize values" << endl;

	// Initialize the first measurement value
	x_ = VectorXd(5);
	x_ << 0,0,0,0,0;

	//cout << "debug1"  << endl;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		float Ro = meas_package.raw_measurements_[0];
		float theta = meas_package.raw_measurements_[1];
		float ro_dot = meas_package.raw_measurements_[2];
		
		x_(0) = Ro * cos(theta);
		x_(1) = Ro * sin(theta);
		x_(2) = 1;
		x_(3) = 1;
		x_(4) = 0;	
	
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		x_(0) = meas_package.raw_measurements_[0];
		x_(1) = meas_package.raw_measurements_[0];
		x_(2) = 1;
		x_(3) = 1;
		x_(4) = 0;
	}
	
  	time_us_ = meas_package.timestamp_;

	//cout << "debug2" << endl;
	// Done initialization
	is_initialized_ = true;
  }

	//cout << "debug3" << endl;
	//call the prediction step
	float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;
 	cout << "Prediction at timestamp" << meas_package.timestamp_ << " with delta_t = " <<  delta_t << endl; 
	UKF::Prediction(delta_t);

	//cout << "debug4" << endl;
	// Call Update step based on LASER or RADAR measurement
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Preform radar update
		cout << "Preform radar update" << endl;
		//UKF::UpdateRadar(meas_package);
	} else { 
		// Preform laser update
		UKF::UpdateLidar(meas_package);
	}
 	//cout << "debug5" << endl;

}

/*
*
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */ 

  cout << "Start prediction step" << endl;
  // size of x state vector
  int n_x_ = 5;

  // size of the augumented state vector with accelration noise and yawdd noise
  int n_aug_ = 7;
  
  double lambda_ = 3 - n_aug_;

  // Predicted sigma
  //MatrixXd Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1); 

  // Augumented state vector
  VectorXd X_aug = VectorXd(7);

  // Augumented start covariance maxtrix
  MatrixXd P_aug = MatrixXd(7, 7);

  X_aug.head(5) = x_;
  X_aug(5) = 0;
  X_aug(6) = 0;
  
  cout << "x_ = " << endl << x_ << endl;
  cout << "X_aug = " << endl <<  X_aug << endl;
 
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;  
  P_aug(6,6) = std_yawdd_ * std_yawdd_;  

  cout << "P_ = " << endl;
  cout << P_ << endl; 
  cout << "P_aug =" << endl;
  cout << P_aug << endl; 

  // Calculate square root of matric
  MatrixXd L = P_aug.llt().matrixL();

  // Create augumented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_,2*n_aug_+1);

  Xsig_aug.col(0) = X_aug;
  cout << "Lambda = " << endl << lambda_ << endl << "n_aug_= " << n_aug_ << endl << "L = " << endl << L << endl;

  for (int i=0; i < n_aug_; i++) {
	Xsig_aug.col(i+1) = X_aug + sqrt(lambda_ + n_aug_) * L.col(i);
	Xsig_aug.col(i+1+n_aug_) = X_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }


  cout << "Xsig_aug = " << endl <<  Xsig_aug << endl;
  cout << "delta_t =" << delta_t << endl;
  //cout << "Xsig_pred = " << endl << Xsig_pred_ << endl;
  // Predict sigma points
  for (int i = 0; i < 2*n_aug_ +1; i++) {
	
	//extract values for readability
	double p_x = Xsig_aug(0,i);
	double p_y = Xsig_aug(1,i);
	double v = Xsig_aug(2,i);
	double yaw = Xsig_aug(3,i);
	double yawd = Xsig_aug(4,i);
	double nu_a = Xsig_aug(5,i);
	double nu_yawdd = Xsig_aug(6,i);	

	// Predicted state values
	double px_p, py_p;

	//avoid division by zero
	if (abs(yawd) > 0.001) {
		px_p = p_x + v/yawd * ( sin(yaw + yaw * delta_t) - sin(yaw));
		py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
	} else {
		px_p = p_x + v * delta_t * cos(yaw);
		py_p = p_y + v * delta_t * sin(yaw);

	}
   	cout << "Print values of predicted px and pt without noise" << endl;
	cout << "px_p = " << endl << px_p << endl;
	cout << "py_p = " << endl << py_p << endl;
	
	double v_p = v;
	double yaw_p = yaw + yawd * delta_t;
	double yawd_p = yawd;

	// add noise
	px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
	py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
	v_p = v_p + nu_a*delta_t;
	yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
	yawd_p = yawd_p + nu_yawdd*delta_t;

	//cout << "debug5" << endl;
	//cout << px_p << endl;
	// write predicted value into right column
	Xsig_pred_(0,i) = px_p;
	Xsig_pred_(1,i) = py_p;
	Xsig_pred_(2,i) = v_p;
	Xsig_pred_(3,i) = yaw_p;
	Xsig_pred_(4,i) = yawd_p;  
	//cout << "debug6" << endl;
  }
  cout << "Xsig_pred = " << endl << Xsig_pred_ << endl;
  // Predict the mean and covariance of the predicted sigma points
  //VectorXd weights = VectorXd(7);

  // set weights
  cout << "lambda = " << lambda_ << endl;
  cout << "n_aug = " << n_aug_ << endl;

  double weight_0 = lambda_/(lambda_+n_aug_);
  //cout << "debug7" << weights_ << endl;
  weights_(0) = weight_0;
  //cout << "debug7" << endl;
  for (int i=1; i<2*n_aug_+1; i++) {
	double weight = 0.5/(lambda_+n_aug_);
	weights_(i) = weight;
 	//cout << "debug8" << endl;
  }  
  cout << "calculate weights = " << endl <<  weights_ << endl;
  //predicted state mean
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
	x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  cout << "Predicted state mean =" << endl << x_ << endl;

  //Predicted state coveriance matrix
  P_.fill(0.0);

  for (int i=0; i < 2 *n_aug_+1; i++) {
	
	//state difference
	VectorXd x_diff = Xsig_pred_.col(i) - x_;
	
	//angle normalization
	while (x_diff(3) > M_PI) x_diff(3)-=2*M_PI;
	while (x_diff(3) < -M_PI) x_diff(3)+=2*M_PI;

	P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  cout << "P_ =" << endl << P_ << endl;
  cout << "End of Prediction step" << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  cout << "Starting Lidar update" << endl;
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  MatrixXd H_ = MatrixXd(2,5);
  H_ << 1, 0, 0, 0, 0,
	0, 1, 0, 0, 0; 

  //Measurement covariance matrix
  MatrixXd R_ = MatrixXd(2,2);
  R_ << std_laspx_*std_laspx_, 0,
	0, std_laspy_*std_laspy_; 
  cout << "debug1" << endl;
  VectorXd y = z - H_ * x_; 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_ ;
  cout << "debug2" << endl;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  x_ = x_ + K * y;

  //size of P_ matrix;
  int size = sizeof P_;
  MatrixXd I = MatrixXd::Identity(5,5);
  cout << "debug3" << endl;
  P_ = (I - K * H_) * P_ ;
  cout << "End of Update Lidar step" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    //transform sigma points into measurement space
  
  cout << " Entering UpdateRadar step" << endl;
  cout << "Xsig_pred" << Xsig_pred_ << endl;
  //set radar measurement dimension
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ +1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  cout << "Covert to radar values" << Zsig << endl;
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  cout << "Predicted output" << z_pred << endl;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  cout << "Measurement covariance maxtrix" << S << endl;

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  cout << "Measurement covariance maxtrix iwth noise" << S << endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Create an vector for radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  cout << "Incoming radar values"<< z << endl;

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  cout << "End of updateRadar step" << endl << "x_=" << x_ << endl << "Process covariance maxtrix =" << P_ << endl;
}
