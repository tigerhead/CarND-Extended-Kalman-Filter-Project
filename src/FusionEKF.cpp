#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {

   cout << "Initializing EKF: " << endl;
   is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  /**
  TODO:
    * Finish initializing the FusionEKF.
  */

  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  R_laser_ << 0.0225, 0,
             0, 0.0225;

  R_radar_ << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;

  //ekf_ <<  KalmanFilter();
  //create a 4D state vector, we don't know yet the values of the x state


  ekf_.x_ = VectorXd(4);

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  //measurement covariance
  ekf_.R_ = MatrixXd(2, 2);
  ekf_.R_ << 0.0225, 0,
            0, 0.0225;

  //measurement matrix
  //ekf_.H_ = MatrixXd(2, 4);
  //ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;



  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        float r = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        //Px
        ekf_.x_[0] =   r * cos(phi);
        //Py
        ekf_.x_[1] =   r * sin(phi);
         cout << "initial position RADAR Measurement: x=" << ekf_.x_[0]  << " y=" << ekf_.x_[1] << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
       //Px
       ekf_.x_[0] =   measurement_pack.raw_measurements_[0] ;
       //Py
       ekf_.x_[1] =   measurement_pack.raw_measurements_[1] ;
       cout << "initial position LIDAR Measurement: x=" << ekf_.x_[0]  << " y=" << ekf_.x_[1] << endl;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  cout << dt << endl;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  ekf_.Q_ = MatrixXd(4, 4);

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
 // cout << "Do prediction" << endl;

  ekf_.Predict();

 // cout << "Prediction Done" << endl;
 // cout << "x_ = " << ekf_.x_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      //Cacluate Jacobian
      //cout << "Calculate Hj" << endl;

      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;

    //  cout << "Hj: " << Hj_ <<endl;

    //  cout << "Do update using EKF" << endl;

      ekf_.UpdateEKF(measurement_pack.raw_measurements_);


  } else {
    // Laser updates
      //Set H_ to linear matrix
      //cout << "Do update using KF" << endl;
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


