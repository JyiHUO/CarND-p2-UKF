#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "fstream"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  eps = 1e-5;

  n_x_ = x_.size();

  n_aug_ = n_x_ + 2;

  lambda_ = 3 - n_aug_;

  Xsig_aug = MatrixXd::Zero(n_aug_, 2*n_aug_+1);

  Xsig_pred_ = MatrixXd::Zero(n_x_, 2*n_aug_+1);

  previous_t = 0;

  x_aug = VectorXd::Zero(n_aug_);

  P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  weights_ = VectorXd::Zero(2*n_aug_+1);

  is_initialized_ = false;

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i=1;i<2*n_aug_+1;i++){
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }


  R_radar = MatrixXd::Zero(3,3);
  R_radar(0,0) = std_radr_*std_radr_;
  R_radar(1,1) = std_radphi_*std_radphi_;
  R_radar(2,2) = std_radrd_*std_radrd_;


  R_lidar = MatrixXd::Zero(2,2);
  R_lidar(0,0) = std_laspx_ * std_laspx_;
  R_lidar(1,1) = std_laspy_ * std_laspy_;

  Q = MatrixXd::Zero(2,2);
  Q<<std_a_*std_a_,0,0,std_yawdd_*std_yawdd_;

  // H matrix for lidar
  H_lidar = MatrixXd::Zero(2,5);
  H_lidar(0,0) = 1;
  H_lidar(1,1) = 1;

  // clear the data file
  ofstream data_file;
  data_file.open("data.txt", ofstream::out | ofstream::trunc);
  data_file.close();
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

  modify parameter which in ukf.h
  */
  if (!is_initialized_){
    P_ = MatrixXd::Identity(n_x_, n_x_);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

    }else{
      x_(1) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      x_(0) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
    }
    previous_t = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package.timestamp_ - previous_t) / 1e6;
  previous_t = meas_package.timestamp_;
  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }else{
    UpdateRadar(meas_package);
  }

  cout<<"nis: "<<nis<<endl;

  // write to file
  ofstream outfile;
  outfile.open("data.txt", ios_base::app);
  outfile << to_string(nis)<<' ';

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
  gen_sigma_point();

  predict_sigma_point(delta_t);

  cal_mean_cov();
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
  predict_lidar_measurement();
  cout<<"predict_lidar_measurement_succ"<<endl;

  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd K = P_ * H_lidar.transpose()*S.inverse();
  x_ += K * y;
  P_ -= K * H_lidar * P_; // K.transpose();

  nis = cal_nis(meas_package);
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
  predict_radar_measurement();
  cout<<"predict_radar_measurement_succ"<<endl;

  Tc = MatrixXd::Zero(n_x_, 3);

  VectorXd sub_x;
  VectorXd sub_z;
  for(int i=0;i<2*n_aug_+1;i++){
    sub_x = Xsig_pred_.col(i) - x_;
    sub_x(1) = atan2(sin(sub_x(1)), cos(sub_x(1)));

    sub_z = Zsig.col(i) - z_pred;
    sub_z(1) = atan2(sin(sub_z(1)), cos(sub_z(1)));
    Tc += weights_(i) * sub_x*sub_z.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //update state mean and covariance matrix
  x_ += K * (meas_package.raw_measurements_ - z_pred);
  P_ -= K * S * K.transpose();

  nis = cal_nis(meas_package);
}

void UKF::gen_sigma_point() {
  // augmentation
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;

  P_aug.topLeftCorner(n_x_, n_x_) = P_;

  P_aug.bottomRightCorner(2,2) = Q;

  MatrixXd A = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_;i++){
    Xsig_aug.col(i+1) = x_aug.array() + sqrt(lambda_ + n_aug_)*A.array().col(i);
    Xsig_aug.col(n_aug_ + i + 1) = x_aug.array() - sqrt(lambda_ + n_aug_)*A.array().col(i);
  }
}

void UKF::predict_sigma_point(double delta_t) {
  for(int i=0;i<Xsig_aug.cols();i++){
    double px = Xsig_aug.col(i)(0);
    double py = Xsig_aug.col(i)(1);
    double v = Xsig_aug.col(i)(2);
    double yaw = Xsig_aug.col(i)(3);
    double yaw_dot = Xsig_aug.col(i)(4);
    double v_dot = Xsig_aug.col(i)(5);
    double yaw_dot_dot = Xsig_aug.col(i)(6);

    // noise vector
    VectorXd noise(n_x_);
    noise<<pow(delta_t,2)*cos(yaw)*v_dot/2,
            pow(delta_t,2)*sin(yaw)*v_dot/2,
            delta_t*v_dot,
            pow(delta_t, 2)*yaw_dot_dot/2,
            delta_t*yaw_dot_dot;

    if(Xsig_aug.col(i)(4)==0){
      px += v*cos(yaw)*delta_t ;
      py += v*sin(yaw)*delta_t ;
      v += 0;
      yaw += 0;
      yaw_dot += 0;
    }else{
      px += v*(sin(yaw + yaw_dot*delta_t) - sin(yaw))/yaw_dot;
      py += v*(-cos(yaw + yaw_dot*delta_t) + cos(yaw))/yaw_dot;
      v += 0;
      yaw += yaw_dot*delta_t;
      yaw_dot += 0;
    }
    Xsig_pred_.col(i) << px, py, v, yaw, yaw_dot;
    Xsig_pred_.col(i) += noise;
  }
}

void UKF::cal_mean_cov(){
  P_ = MatrixXd::Zero(n_x_, n_x_);
  x_ = Xsig_pred_ * weights_;

  for(int i=0;i<2*n_aug_+1;i++){
    VectorXd subtract = Xsig_pred_.col(i) - x_;
    subtract(3) = atan2(sin(subtract(3)),cos(subtract(3)));
    P_ += weights_(i) * subtract*subtract.transpose();
  }

}

void UKF::predict_radar_measurement() {
  // UKF
  // initialization

  Zsig = MatrixXd::Zero(3, 2 * n_aug_ + 1);
  z_pred = VectorXd::Zero(3);

  //measurement covariance matrix S
  S = MatrixXd::Zero(3,3);

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
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / max(sqrt(p_x*p_x + p_y*p_y), eps);   //r_dot
  }
  //calculate mean predicted measurement
  for(int i=0;i<2*n_aug_ + 1;i++){
    z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  for(int i=0;i<2*n_aug_ + 1;i++){
    VectorXd sub = Zsig.col(i) - z_pred;

    // norm
    sub(1) = atan2(sin(sub(1)),cos(sub(1)));
    S+= weights_(i)*sub*sub.transpose();
  }
  S+=R_radar;
}

void UKF::predict_lidar_measurement() {
  // EKF
  S = MatrixXd::Zero(2,2);

  z_pred = H_lidar * x_;
  S = H_lidar * P_ * H_lidar.transpose() + R_lidar;
}

double UKF::cal_nis(MeasurementPackage mp) {
  VectorXd sub = mp.raw_measurements_ - z_pred;
  double nis_ = sub.transpose() * S.inverse() * sub;
  return nis_;
}