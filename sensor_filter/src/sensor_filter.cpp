/*
 * sensor_filter.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#include "sensor_filter/sensor_filter.h"


// low pass filter /////////////////////////////////////////////////////////
/*
LowPassFilter::LowPassFilter(double control_time_init, double cutoff_frequency_init)
:control_time(control_time_init),cutoff_frequency(cutoff_frequency_init)
{
  this->initialize();
}*/
LowPassFilter::LowPassFilter()
{
  lambda = 0.0;
  alpha = 0.0;
}
LowPassFilter::~LowPassFilter()
{
}
void LowPassFilter::initialize()
{
  raw_data = 0.0;
  pre_filtered_data = 0.0;

  lambda = 1/(2*M_PI*cutoff_frequency);
  alpha = control_time/(lambda + control_time);
}
double LowPassFilter::lpf_processing(double input_data)
{
  static double filtered_data = 0.0;

  filtered_data = alpha*input_data + (1-alpha)*pre_filtered_data;

  pre_filtered_data = filtered_data;

  return filtered_data;
}

////////////////////////////////////////////////////////////////////////////

// high pass filter /////////////////////////////////////////////////////////
/*
HighPassFilter::HighPassFilter(double control_time_init, double cutoff_frequency_init)
:control_time(control_time_init),cutoff_frequency(cutoff_frequency_init)
{
  this->initialize();
}*/
HighPassFilter::HighPassFilter()
{
  lambda = 0.0;
  alpha = 0.0;
}
HighPassFilter::~HighPassFilter()
{
}
void HighPassFilter::initialize()
{
  raw_data = 0.0;
  pre_input_data = 0.0;
  pre_filtered_data = 0.0;

  lambda = 1/(2*M_PI*cutoff_frequency);
  alpha = lambda/(lambda + control_time);
}
double HighPassFilter::hpf_processing(double input_data)
{
  static double filtered_data = 0.0;

  filtered_data = alpha*pre_filtered_data + alpha*(input_data - pre_input_data);

  pre_filtered_data = filtered_data;
  pre_input_data    = input_data;

  return filtered_data;
}

////////////////////////////////////////////////////////////////////////////

// kalman filter ///////////////////////////////////////////////////////////

KalmanFilter::KalmanFilter()
{

}
KalmanFilter::~KalmanFilter()
{

}
void KalmanFilter::initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables)
{
  // must be designed by your system model
  F.resize(state_variables.rows(),state_variables.rows());
  H.resize(measurement_variables.rows(),state_variables.rows());
  Q.resize(state_variables.rows(),state_variables.rows());
  R.resize(measurement_variables.rows(),measurement_variables.rows());

  F.setIdentity();
  H.setIdentity();
  Q.setIdentity();
  R.setIdentity();

  //intial condition
  correction_value_x.resize(state_variables.rows(),state_variables.cols());
  correction_value_p.resize(state_variables.rows(),state_variables.rows());

  correction_value_x.fill(0);
  correction_value_p.setIdentity();

  //variables
  prediction_value_x.resize(state_variables.rows(),state_variables.cols());
  prediction_value_p.resize(state_variables.rows(),state_variables.rows());

  prediction_value_x.fill(0);
  prediction_value_p.setIdentity();

  previous_correction_value_x.resize(state_variables.rows(),state_variables.cols());
  previous_correction_value_p.resize(state_variables.rows(),state_variables.rows());

  previous_correction_value_x.fill(0);
  previous_correction_value_p.setIdentity();

  previous_correction_value_x = correction_value_x;
  previous_correction_value_p = correction_value_p;

  //kalman gain
  kalman_gain_k.resize(state_variables.rows(),state_variables.rows());
  kalman_gain_k.fill(0);
}
Eigen::MatrixXd KalmanFilter::kalman_filtering_processing(Eigen::MatrixXd measurement_z)
{


  prediction_value_x = F * previous_correction_value_x;

  prediction_value_p = F * correction_value_p * F.transpose() + Q;

  if((H * prediction_value_p * H.transpose() + R).determinant() == 0)
  {
    return correction_value_x;
  }

  kalman_gain_k = prediction_value_p * H.transpose() * ((H * prediction_value_p * H.transpose() + R).inverse());

  correction_value_x = prediction_value_x + kalman_gain_k * (measurement_z - (H * prediction_value_x));

  correction_value_p = prediction_value_p - (kalman_gain_k * H * prediction_value_p);

  previous_correction_value_x = correction_value_x;
  previous_correction_value_p = correction_value_p;

  return correction_value_x;
}
////////////////////////////////////////////////////////////////////////////

// Kalman Bucy Filter filter ///////////////////////////////////////////////

KalmanBucyFilter::KalmanBucyFilter()
{

}
KalmanBucyFilter::~KalmanBucyFilter()
{

}
void KalmanBucyFilter::initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables)
{
  // must be designed by your system model
  F.resize(state_variables.rows(),state_variables.rows());
  H.resize(measurement_variables.rows(),state_variables.rows());
  Q.resize(state_variables.rows(),state_variables.rows());
  R.resize(measurement_variables.rows(),measurement_variables.rows());

  F.setIdentity();
  H.setIdentity();
  Q.setIdentity();
  R.setIdentity();

  //intial condition
  estimated_value_x.resize(state_variables.rows(),state_variables.cols());
  estimated_value_p.resize(state_variables.rows(),state_variables.rows());

  estimated_value_x.fill(0);
  estimated_value_p.setIdentity();

  //variables
  dt_value_x.resize(state_variables.rows(),state_variables.cols());
  dt_value_p.resize(state_variables.rows(),state_variables.rows());

  dt_value_x.fill(0);
  dt_value_p.setIdentity();

  pre_dt_value_x.resize(state_variables.rows(),state_variables.cols());
  pre_dt_value_p.resize(state_variables.rows(),state_variables.rows());

  pre_dt_value_x.fill(0);
  pre_dt_value_p.setIdentity();

  //kalman gain
  kalman_gain_k.resize(state_variables.rows(),state_variables.rows());
  kalman_gain_k.fill(0);

}

Eigen::MatrixXd KalmanBucyFilter::kalman_bucy_filtering_processing(Eigen::MatrixXd measurement_z)
{

  kalman_gain_k = estimated_value_p * H.transpose() * R.inverse();

  dt_value_x = F * estimated_value_x + kalman_gain_k * (measurement_z - (H * estimated_value_x));
  //
  dt_value_p = F * estimated_value_p + estimated_value_p * F.transpose() - (kalman_gain_k * R * kalman_gain_k.transpose()) + Q;
  //
  estimated_value_x += ((pre_dt_value_x + dt_value_x)*control_time)/2;
  estimated_value_p += ((pre_dt_value_p + dt_value_p)*control_time)/2; // trapezoidal method integral
  //
  pre_dt_value_x = dt_value_x;
  pre_dt_value_p = dt_value_p;

  return estimated_value_x;
}

////////////////////////////////////////////////////////////////////////////
