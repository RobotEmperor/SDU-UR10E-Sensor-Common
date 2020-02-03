/*
 * ur10e_force_torque_sensor.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef SDU_UR10E_SENSOR_UR10E_FORCE_TORQUE_SENSOR_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_UR10E_FORCE_TORQUE_SENSOR_H_
#define SDU_UR10E_SENSOR_UR10E_FORCE_TORQUE_SENSOR_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_UR10E_FORCE_TORQUE_SENSOR_H_

#include <stdio.h>
#include <math.h>

#include "sensor_filter/sensor_filter.h"
#include "sdu_math/kinematics.h"
#include "sdu_math/statistics_math.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>



class Ur10eFTsensor
{
  public:
    Ur10eFTsensor();
    ~Ur10eFTsensor();
    void initialize();
    void offset_init(Eigen::MatrixXd data, bool time_check);
    void signal_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz
    void parse_init_data(const std::string &path);

    void collision_detection_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz

    //initialize
    double control_time;
    double lpf_force_cutoff_frequency;
    double lpf_torque_cutoff_frequency;

    double hpf_force_cutoff_frequency;
    double hpf_torque_cutoff_frequency;

    double fx_detection, fx_k, fx_high_limit, fx_low_limit;
    double fy_detection, fy_k, fy_high_limit, fy_low_limit;
    double fz_detection, fz_k, fz_high_limit, fz_low_limit;
    double tx_detection, tx_k, tx_high_limit, tx_low_limit;
    double ty_detection, ty_k, ty_high_limit, ty_low_limit;
    double tz_detection, tz_k, tz_high_limit, tz_low_limit;


    double ft_raw_data;

    //filter will be added
    LowPassFilter *low_pass_filter_fx;
    LowPassFilter *low_pass_filter_fy;
    LowPassFilter *low_pass_filter_fz;

    LowPassFilter *low_pass_filter_tx;
    LowPassFilter *low_pass_filter_ty;
    LowPassFilter *low_pass_filter_tz;

    HighPassFilter *high_pass_filter_fx;
    HighPassFilter *high_pass_filter_fy;
    HighPassFilter *high_pass_filter_fz;

    HighPassFilter *high_pass_filter_tx;
    HighPassFilter *high_pass_filter_ty;
    HighPassFilter *high_pass_filter_tz;

    KalmanFilter  *kalman_filter_force_torque;
    KalmanFilter  *kalman_filter_force_torque_temp;
    KalmanBucyFilter  *kalman_bucy_filter_force_torque;

    double gain_q;
    double gain_r_low_frequency;
    double gain_r_high_frequency;

    double limit_low_rate_of_change;
    double limit_high_rate_of_change;


    Eigen::MatrixXd ft_filtered_data;

    Eigen::MatrixXd ft_filtered_data_temp;

    Eigen::MatrixXd ft_offset_data;

    Eigen::MatrixXd pre_ft_filtered_data;
    Eigen::MatrixXd rate_of_change_ft_filtered_data;


  private:
};

class PoseEstimation
{
  public:
    PoseEstimation();
    ~PoseEstimation();
    void initialize();
    void estimation_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz
    void offset_init(Eigen::MatrixXd data, bool time_check);

    //initialize

    double control_time;
    double mass_of_tool;

    Eigen::MatrixXd tool_pose_data;
    Eigen::MatrixXd tool_vel_data;
    Eigen::MatrixXd tool_acc_data;

    Eigen::MatrixXd pre_tool_pose_data;
    Eigen::MatrixXd pre_tool_vel_data;
    Eigen::MatrixXd pre_tool_acc_data;

    Eigen::MatrixXd tool_linear_acc_data;

    Eigen::MatrixXd inertia_of_tool;
    Eigen::MatrixXd contacted_force_torque;

    //filter
    KalmanFilter  *kalman_filter_linear_acc;

    Eigen::MatrixXd filtered_data;
    Eigen::MatrixXd offset_data;

  private:
};



#endif /* SDU_UR10E_SENSOR_UR10E_FORCE_TORQUE_SENSOR_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_UR10E_FORCE_TORQUE_SENSOR_H_ */
