#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

   cout << "inside initialization" << endl;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

   is_initialized_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;



    // set state dimension
    n_x_ = 5;

  // initial state vector
   x_ = VectorXd(n_x_);

    // define spreading parameter
    lambda = 3 - n_aug_;



  // initial covariance matrix
  P_ = MatrixXd(5, 5);


  time_us_ =0;

   time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // change this
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // change this
    std_yawdd_ = 30;


   // Change these back to their original values
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

}

/**
 * TODO Store Laser and Radar NIS, modify std_a and std_yawdd, currently too high
 * Update step for LIdar -- predict measurement, update state
 *
 *
 */

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


  // Initialization structure similar to EKF project

  if (!is_initialized_){


    // Initialize (state) x_, P_, previous_time, anything else needed

      x_ <<   5.7441,
              1.3800,
              2.2049,
              0.5015,
              0.3528;

      P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
              -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
              0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
              -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
              -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

          // Convert radar from polar to cartesian coordinates and initialize state.


          float ro = meas_package.raw_measurements_[0];
          float psi = meas_package.raw_measurements_[1];
          float ro_dot = meas_package.raw_measurements_[2];


          x_ << ro * cos(psi), ro * sin(psi), ro_dot, psi, 0;

      }

  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    // Initialize here

      x_ <<  meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
  }

  // Initialize anything else here (e.g. P_, anything else needed)
  time_us_ = meas_package.timestamp_;
  is_initialized_ = true;
  return;
  }
 
  // Control structure similar to EKF project 

  float delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  Prediction(delta_t);


  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  time_us_ = meas_package.timestamp_;


}

/**
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

  // Create Augmented Sigma Points

  // Lesson 7, section 18: Augmentation Assignment 2


   // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x_, 2 *n_x_+1);
  // create example matrix with predicted sigma points


     // move into separate method void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out)
     // = augmented state
  // Xsig_pred = Xsig_aug


    // set example state

    VectorXd x = VectorXd(n_x_);

    x <<   5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    //set example covariance matrix
    MatrixXd P = MatrixXd(n_x_, n_x_);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // create sigma point matrix
    MatrixXd Xsig_pred = MatrixXd(n_aug_, 2 *n_aug_+1);

    // calculate square root of P
    MatrixXd A = P_.llt().matrixL();

    //set first column of sigma point matrix
    Xsig.col(0)  = x;

    //set remaining sigma points
    for (int i = 0; i < n_x_; i++)
    {
        Xsig.col(i+1)     = x + sqrt(lambda+n_x_) * A.col(i);
        Xsig.col(i+1+n_x_) = x - sqrt(lambda+n_x_) * A.col(i);
    }

    /// end generate sigma points




    x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)      = x_aug + sqrt(lambda+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_)  = x_aug - sqrt(lambda+n_aug_) * L.col(i);
  }

    // end void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out)


    // move into separate method  void UKF::SigmaPointPrediction(MatrixXd* Xsig_out)
  // Predict Sigma Points

  // Lesson 7, section 21 Sigma Point Prediction Assignment 2
  // predicted state will have 5 rows and fifteen columns
  // augmented state will have 7 rows and 15 columns
  // predict sigma points

    //create example sigma point matrix
   // MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug <<
             5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
            1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
            2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
            0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
            0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
            0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
            0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  for (int i = 0; i < 2*n_aug_+1; i++){
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
    }

    // end sigma point prediction void UKF::SigmaPointPrediction(MatrixXd* Xsig_out)


    // Move into new method void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out)
    // Predict mean and covariance

    // maybe need to change these values, create different values for each method

    //create example matrix with predicted sigma points
    Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);

    // Lesson 8, 24
    // set weights
    double weight_0 = lambda/(lambda+n_aug_);
    weights(0) = weight_0;
    for (int i=1; i < 2*n_aug_+1; i++){ //2n+1 weights
      double weight = 0.5/(n_aug_+lambda);
      weights(i) = weight;
    }

    // predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points
      x_ = x_ + weights[i] * Xsig_pred.col(i);
    }

    // predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ +1; i++){ // iterate overy sigma points
      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x_;
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      P_ = P_ + weights(i) * x_diff * x_diff.transpose();
    }

    // end void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out)

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

  // Update Lidar Measurement

  // The mapping from state space to Lidar is linear. Fill this out with
  // appropriate update steps

  // Fix this part ****

    if (!is_initialized_){


        // set the state with the intial location and zero velocity
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0,0;

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;

    }


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

  // Change variable names

  // Predict Radar Sigma Points

  // transform sigma points into measurement space


  // Lesson 8 part 27
  // move into two separate methods
  //  void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out)
  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

     for (int i = 0; i < 2 * n_aug_ +1; i++) { // 2n+1 sigma points

     // extract values for better readibility
     double p_x = Xsig_pred(0,i);
     double p_y = Xsig_pred(1,i);
        double v  = Xsig_pred(2,i);
        double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);         // r
    Zsig(1,i) = atan2(p_y, p_x);                 //  phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y); // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);


  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

      // measurement covariance matrix S
      MatrixXd S = MatrixXd(n_z_, n_z_);
      S.fill(0.0);
      for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n + 1 sigma points
          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;

          // angle normalization
          while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
          while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

          S = S + weights(i) * z_diff * z_diff.transpose();
      }

      // add measurement noise covariance matrix
      MatrixXd R = MatrixXd(n_z_, n_z_);
      R << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;
      S = S + R;

    // end predict radar

      // Update State, move into a separate method
      // void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out)
      // Lesson 8, section 30: UKF Update Assignment 2
      // add example values
      // create matrix for cross correlation Tc
      MatrixXd Tc = MatrixXd(n_x_, n_z_);

      // calculate cross correlation matrix
      Tc.fill(0.0);
      for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 sigma points

          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;
          // angle normalization
          while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
          while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

          // state difference
          VectorXd x_diff = Xsig_pred.col(i) - x_;
          // angle normalization
          while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
          while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

          Tc = Tc + weights(i) * x_diff * z_diff.transpose();
      }

      // Kalman gain K;
      MatrixXd K = Tc * S.inverse();


      // create example vector for incoming radar measurement
      VectorXd z = VectorXd(n_z_);
      z <<  5.9214, 0.2187, 2.0062;


      // residual
      VectorXd z_diff = z - z_pred;

      // angle normalization
      while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

      // update state mean and covariance matrix
      x_ = x_ + K * z_diff;
      P_ = P_ - K * S * K.transpose();



  }
