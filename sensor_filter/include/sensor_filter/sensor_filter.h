/*
 * sensor_filter.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_
#define SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "sdu_math/statistics_math.h"


class LowPassFilter // lpf
{
  public:
    //LowPassFilter(double control_time_init, double cutoff_frequency_init);
    LowPassFilter();
    ~LowPassFilter();
    void initialize();
    double lpf_processing(double input_data); // frq = frequency , ctrl = control
    double control_time, cutoff_frequency;

  private:

    double raw_data;
    double pre_filtered_data;
    double lambda;
    double alpha;
};

class HighPassFilter // hpf
{
  public:
    //LowPassFilter(double control_time_init, double cutoff_frequency_init);
    HighPassFilter();
    ~HighPassFilter();
    void initialize();
    double hpf_processing(double input_data); // frq = frequency , ctrl = control
    double control_time, cutoff_frequency;

  private:

    double raw_data;
    double pre_input_data;
    double pre_filtered_data;
    double lambda;
    double alpha;
};



class KalmanFilter // kf
{
  public:
    KalmanFilter();
    ~KalmanFilter();
    void initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables);

    // must be designed by your system model
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    //intial condition
    Eigen::MatrixXd correction_value_x;
    Eigen::MatrixXd correction_value_p;

    //kalman filter process
    Eigen::MatrixXd kalman_filtering_processing(Eigen::MatrixXd measurement_z);

  private:
    //variables
    Eigen::MatrixXd prediction_value_x;
    Eigen::MatrixXd prediction_value_p;

    Eigen::MatrixXd previous_correction_value_x;
    Eigen::MatrixXd previous_correction_value_p;

    //kalman gain
    Eigen::MatrixXd kalman_gain_k;

    double raw_data;

};

class KalmanBucyFilter // kbf
{
  public:
    KalmanBucyFilter();
    ~KalmanBucyFilter();
    void initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables);
    // must be designed by your system model

    double control_time;

    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::MatrixXd estimated_value_x;
    Eigen::MatrixXd estimated_value_p;


    Eigen::MatrixXd kalman_bucy_filtering_processing(Eigen::MatrixXd measurement_z);

  private:
    //variables

    Eigen::MatrixXd dt_value_x;
    Eigen::MatrixXd dt_value_p;

    Eigen::MatrixXd pre_dt_value_x;
    Eigen::MatrixXd pre_dt_value_p;

    //kalman gain
    Eigen::MatrixXd kalman_gain_k;

    double raw_data;

};
#endif /* SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_ */
