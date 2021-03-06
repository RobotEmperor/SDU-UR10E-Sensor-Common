/*
 * ur10e_force_torque_sensor.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#include "ur10e_force_torque_sensor/ur10e_force_torque_sensor.h"


Ur10eFTsensor::Ur10eFTsensor()
{
  low_pass_filter_fx  = std::make_shared<LowPassFilter>();
  low_pass_filter_fy  = std::make_shared<LowPassFilter>();
  low_pass_filter_fz  = std::make_shared<LowPassFilter>();
  low_pass_filter_tx  = std::make_shared<LowPassFilter>();
  low_pass_filter_ty  = std::make_shared<LowPassFilter>();
  low_pass_filter_tz  = std::make_shared<LowPassFilter>();
  high_pass_filter_fx = std::make_shared<HighPassFilter>();
  high_pass_filter_fy = std::make_shared<HighPassFilter>();
  high_pass_filter_fz = std::make_shared<HighPassFilter>();
  high_pass_filter_tx = std::make_shared<HighPassFilter>();
  high_pass_filter_ty = std::make_shared<HighPassFilter>();
  high_pass_filter_tz = std::make_shared<HighPassFilter>();

  kalman_filter_force_torque = std::make_shared<KalmanFilter>();
  kalman_filter_force_torque_temp = std::make_shared<KalmanFilter>();
  kalman_bucy_filter_force_torque = std::make_shared<KalmanBucyFilter>();
  ft_contact = std::make_shared<KalmanFilter>();


  control_time = 0;
  lpf_force_cutoff_frequency  = 0;
  lpf_torque_cutoff_frequency = 0;
  hpf_force_cutoff_frequency  = 0;
  hpf_torque_cutoff_frequency = 0;

  gain_q = 0;
  gain_r_low_frequency = 0;
  gain_r_high_frequency = 0;

  gain_r_torque_low_frequency = 0;
  gain_r_torque_high_frequency = 0;


  limit_low_rate_of_change = 0;
  limit_high_rate_of_change = 0;

  fx_detection = 0; fx_k = 0; fx_high_limit = 0; fx_low_limit = 0;
  fy_detection = 0; fy_k = 0; fy_high_limit = 0; fy_low_limit = 0;
  fz_detection = 0; fz_k = 0; fz_high_limit = 0; fz_low_limit = 0;
  tx_detection = 0; tx_k = 0; tx_high_limit = 0; tx_low_limit = 0;
  ty_detection = 0; ty_k = 0; ty_high_limit = 0; ty_low_limit = 0;
  tz_detection = 0; tz_k = 0; tz_high_limit = 0; tz_low_limit = 0;

  initialize();

}

Ur10eFTsensor::~Ur10eFTsensor()
{
}
void Ur10eFTsensor::parse_init_data(const std::string &path)
{
  YAML::Node doc; // YAML file class 선언!
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

  }catch(const std::exception& e) // 에러 점검
  {
    printf("Fail to load yaml file!");
    return;
  }
  control_time = doc["control_time"].as<double>();
  lpf_force_cutoff_frequency = doc["lpf_force_cutoff_frequency"].as<double>();
  lpf_torque_cutoff_frequency = doc["lpf_torque_cutoff_frequency"].as<double>();

  hpf_force_cutoff_frequency = doc["hpf_force_cutoff_frequency"].as<double>();
  hpf_torque_cutoff_frequency = doc["hpf_torque_cutoff_frequency"].as<double>();

  gain_q = doc["gain_Q"].as<double>();
  gain_r_low_frequency = doc["gain_R_low_frequency"].as<double>();
  gain_r_high_frequency = doc["gain_R_high_frequency"].as<double>();

  gain_r_torque_low_frequency = doc["gain_R_torque_low_frequency"].as<double>();
  gain_r_torque_high_frequency = doc["gain_R_torque_high_frequency"].as<double>();

  limit_low_rate_of_change = doc["limit_low_rate_of_change"].as<double>();
  limit_high_rate_of_change = doc["limit_high_rate_of_change"].as<double>();

  fx_k = doc["fx_k"].as<double>();
  fy_k = doc["fy_k"].as<double>();
  fz_k = doc["fz_k"].as<double>();
  tx_k = doc["tx_k"].as<double>();
  ty_k = doc["ty_k"].as<double>();
  tz_k = doc["tz_k"].as<double>();

  fx_high_limit = doc["fx_high_limit"].as<double>();
  fy_high_limit = doc["fy_high_limit"].as<double>();
  fz_high_limit = doc["fz_high_limit"].as<double>();
  tx_high_limit = doc["tx_high_limit"].as<double>();
  ty_high_limit = doc["ty_high_limit"].as<double>();
  tz_high_limit = doc["tz_high_limit"].as<double>();

  fx_low_limit = doc["fx_low_limit"].as<double>();
  fy_low_limit = doc["fy_low_limit"].as<double>();
  fz_low_limit = doc["fz_low_limit"].as<double>();
  tx_low_limit = doc["tx_low_limit"].as<double>();
  ty_low_limit = doc["ty_low_limit"].as<double>();
  tz_low_limit = doc["tz_low_limit"].as<double>();
}
void Ur10eFTsensor::initialize()
{
  ft_raw_data.resize(6,1);
  ft_raw_data.fill(0);

  ft_filtered_data.resize(6, 1);
  ft_filtered_data.fill(0);

  ft_filtered_data_temp.resize(6, 1);
  ft_filtered_data_temp.fill(0);

  pre_ft_filtered_data.resize(6,1);
  pre_ft_filtered_data.fill(0);

  rate_of_change_ft_filtered_data.resize(6,1);
  rate_of_change_ft_filtered_data.fill(0);

  ft_offset_data.resize(6,1);
  ft_offset_data.fill(0);

  low_pass_filter_fx->control_time = control_time;
  low_pass_filter_fy->control_time = control_time;
  low_pass_filter_fz->control_time = control_time;
  low_pass_filter_tx->control_time = control_time;
  low_pass_filter_ty->control_time = control_time;
  low_pass_filter_tz->control_time = control_time;
  low_pass_filter_fx->cutoff_frequency = lpf_force_cutoff_frequency;
  low_pass_filter_fy->cutoff_frequency = lpf_force_cutoff_frequency;
  low_pass_filter_fz->cutoff_frequency = lpf_force_cutoff_frequency;
  low_pass_filter_tx->cutoff_frequency = lpf_torque_cutoff_frequency;
  low_pass_filter_ty->cutoff_frequency = lpf_torque_cutoff_frequency;
  low_pass_filter_tz->cutoff_frequency = lpf_torque_cutoff_frequency;

  low_pass_filter_fx->initialize();
  low_pass_filter_fy->initialize();
  low_pass_filter_fz->initialize();
  low_pass_filter_tx->initialize();
  low_pass_filter_ty->initialize();
  low_pass_filter_tz->initialize();


  high_pass_filter_fx->control_time = control_time;
  high_pass_filter_fy->control_time = control_time;
  high_pass_filter_fz->control_time = control_time;
  high_pass_filter_tx->control_time = control_time;
  high_pass_filter_ty->control_time = control_time;
  high_pass_filter_tz->control_time = control_time;
  high_pass_filter_fx->cutoff_frequency = hpf_force_cutoff_frequency;
  high_pass_filter_fy->cutoff_frequency = hpf_force_cutoff_frequency;
  high_pass_filter_fz->cutoff_frequency = hpf_force_cutoff_frequency;
  high_pass_filter_tx->cutoff_frequency = hpf_torque_cutoff_frequency;
  high_pass_filter_ty->cutoff_frequency = hpf_torque_cutoff_frequency;
  high_pass_filter_tz->cutoff_frequency = hpf_torque_cutoff_frequency;

  high_pass_filter_fx->initialize();
  high_pass_filter_fy->initialize();
  high_pass_filter_fz->initialize();
  high_pass_filter_tx->initialize();
  high_pass_filter_ty->initialize();
  high_pass_filter_tz->initialize();


  kalman_filter_force_torque->initialize(ft_filtered_data,ft_filtered_data);
  kalman_filter_force_torque->F.setIdentity();
  kalman_filter_force_torque->H.setIdentity();
  kalman_filter_force_torque->Q.setIdentity();
  kalman_filter_force_torque->R.setIdentity();

  kalman_filter_force_torque->Q = kalman_filter_force_torque->Q*gain_q; // sensor noise filtering  --> can be modified a external file.
  kalman_filter_force_torque->R = kalman_filter_force_torque->R*gain_r_low_frequency; // sensor noise filtering  --> can be modified a external file.

  kalman_filter_force_torque_temp->initialize(ft_filtered_data_temp,ft_filtered_data_temp);
  kalman_filter_force_torque_temp->F.setIdentity();
  kalman_filter_force_torque_temp->H.setIdentity();
  kalman_filter_force_torque_temp->Q.setIdentity();
  kalman_filter_force_torque_temp->R.setIdentity();

  kalman_filter_force_torque_temp->Q = kalman_filter_force_torque_temp->Q*gain_q; // sensor noise filtering  --> can be modified a external file.
  kalman_filter_force_torque_temp->R = kalman_filter_force_torque_temp->R*gain_r_low_frequency; // sensor noise filtering  --> can be modified a external file.

  kalman_bucy_filter_force_torque->initialize(ft_filtered_data,ft_filtered_data);
  kalman_bucy_filter_force_torque->F.setIdentity();
  kalman_bucy_filter_force_torque->H.setIdentity();
  kalman_bucy_filter_force_torque->Q.setIdentity();
  kalman_bucy_filter_force_torque->R.setIdentity();

  //kalman_bucy_filter_force_torque->R = kalman_bucy_filter_force_torque->R*0.003; // sensor noise filtering  --> can be modified a external file.

  kalman_bucy_filter_force_torque->control_time = control_time;


  ft_contact ->B.setIdentity();
  ft_contact ->initialize(ft_filtered_data,ft_filtered_data);
  ft_contact ->F.setIdentity();
  ft_contact ->H.setIdentity();
  ft_contact ->Q.setIdentity();
  ft_contact ->R.setIdentity();
  ft_contact ->Q = ft_contact->Q*0.001; // sensor noise filtering  --> can be modified a external file.
  ft_contact ->R = ft_contact->R*2; // sensor noise filtering  --> can be modified a external file.

}
void Ur10eFTsensor::offset_init(Eigen::MatrixXd data, int desired_sample_num)
{
  static int sample_num = 1;

  ft_offset_data += data;

  if(sample_num == desired_sample_num)
  {
    ft_offset_data = ft_offset_data/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}
void Ur10eFTsensor::signal_processing(Eigen::MatrixXd data)
{
  //kalman filer

  data = data - ft_offset_data;
  ft_filtered_data = kalman_filter_force_torque->kalman_filtering_processing(data);


  rate_of_change_ft_filtered_data = (ft_filtered_data - pre_ft_filtered_data)*(1/control_time);


  pre_ft_filtered_data = ft_filtered_data;

  rate_of_change_ft_filtered_data(0,0) = low_pass_filter_fx->lpf_processing(rate_of_change_ft_filtered_data(0,0));
  rate_of_change_ft_filtered_data(1,0) = low_pass_filter_fy->lpf_processing(rate_of_change_ft_filtered_data(1,0));
  rate_of_change_ft_filtered_data(2,0) = low_pass_filter_fz->lpf_processing(rate_of_change_ft_filtered_data(2,0));

  rate_of_change_ft_filtered_data(3,0) = low_pass_filter_tx->lpf_processing(rate_of_change_ft_filtered_data(3,0));
  rate_of_change_ft_filtered_data(4,0) = low_pass_filter_ty->lpf_processing(rate_of_change_ft_filtered_data(4,0));
  rate_of_change_ft_filtered_data(5,0) = low_pass_filter_tz->lpf_processing(rate_of_change_ft_filtered_data(5,0));

  for(int num = 0; num < 3; num ++)
  {
    if(rate_of_change_ft_filtered_data(num,0) >  limit_high_rate_of_change || rate_of_change_ft_filtered_data(num,0) < limit_low_rate_of_change)
    {
      kalman_filter_force_torque->R(num,num) = gain_r_high_frequency;
    }
    else
    {
      kalman_filter_force_torque->R(num,num) = gain_r_low_frequency;
    }
  }
  for(int num = 3; num < 6; num ++)
  {
    if(rate_of_change_ft_filtered_data(num,0) >  limit_high_rate_of_change*0.15 || rate_of_change_ft_filtered_data(num,0) < limit_low_rate_of_change*0.15)
    {
      kalman_filter_force_torque->R(num,num) = gain_r_torque_high_frequency;
    }
    else
    {
      kalman_filter_force_torque->R(num,num) = gain_r_torque_low_frequency;
    }
  }

 // ft_filtered_data_temp = kalman_filter_force_torque_temp->kalman_filtering_processing(data);




  //ft_filtered_data_temp = ft_contact->kalman_filtering_processing(ft_filtered_data_temp);



  //kalman bucy filer
  //ft_filtered_data = kalman_bucy_filter_force_torque->kalman_bucy_filtering_processing(data);

  //lospass filter
  //ft_filtered_data(0,0) = low_pass_filter_fx->lpf_processing(ft_filtered_data(0,0));
  //ft_filtered_data(1,0) = low_pass_filter_fy->lpf_processing(data(1,0));
  //ft_filtered_data(2,0) = low_pass_filter_fz->lpf_processing(data(2,0));
  //ft_filtered_data(3,0) = low_pass_filter_tx->lpf_processing(data(3,0));
  //ft_filtered_data(4,0) = low_pass_filter_ty->lpf_processing(data(4,0));
  //ft_filtered_data(5,0) = low_pass_filter_tz->lpf_processing(data(5,0));

  //higpass filter
  //ft_filtered_data(0,0) = high_pass_filter_fx->hpf_processing(data(0,0));
  //ft_filtered_data(1,0) = high_pass_filter_fy->hpf_processing(data(1,0));
  //ft_filtered_data(2,0) = high_pass_filter_fz->hpf_processing(data(2,0));
  //ft_filtered_data(3,0) = high_pass_filter_tx->hpf_processing(data(3,0));
  //ft_filtered_data(4,0) = high_pass_filter_ty->hpf_processing(data(4,0));
  //ft_filtered_data(5,0) = high_pass_filter_tz->hpf_processing(data(5,0));
}

// collision detection
void Ur10eFTsensor::collision_detection_processing(Eigen::MatrixXd data)
{
  fx_detection = calculate_cusum(data(0,0),fx_k,fx_high_limit,fx_low_limit); // how to decide k, limit
  fy_detection = calculate_cusum(data(1,0),fy_k,fy_high_limit,fy_low_limit);
  fz_detection = calculate_cusum(data(2,0),fz_k,fz_high_limit,fz_low_limit);
  tx_detection = calculate_cusum(data(3,0),tx_k,tx_high_limit,tx_low_limit);
  ty_detection = calculate_cusum(data(4,0),ty_k,ty_high_limit,ty_low_limit);
  tz_detection = calculate_cusum(data(5,0),tz_k,tz_high_limit,tz_low_limit);
}

// pose estimation

PoseEstimation::PoseEstimation()
{
  initialize();
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::initialize()
{
  control_time = 0.0;
  mass_of_tool = 4.1118; // must be changed
  tool_pose_data.resize(6,1);
  tool_vel_data.resize(6,1);
  tool_acc_data.resize(6,1);

  pre_tool_pose_data.resize(6,1);
  pre_tool_vel_data.resize(6,1);
  pre_tool_acc_data.resize(6,1);

  tool_linear_acc_data.resize(6,1);

  inertia_of_tool.resize(3,3);
  contacted_force_torque.resize(6,1);

  //////////////////////////////////
  tool_pose_data.fill(0);
  tool_vel_data.fill(0);
  tool_acc_data.fill(0);

  pre_tool_pose_data.fill(0);
  pre_tool_vel_data.fill(0);
  pre_tool_acc_data.fill(0);

  tool_linear_acc_data.fill(0);

  inertia_of_tool.fill(0);
  contacted_force_torque.fill(0);
  //////////////////////////////////

  //kalman_filter_linear_acc = new KalmanFilter; // for linear acc sensor
  kalman_filter_linear_acc = std::make_shared<KalmanFilter>();

  filtered_data.resize(6,1);
  filtered_data.fill(0);

  offset_data.resize(6,1);
  offset_data.fill(0);

  kalman_filter_linear_acc->initialize(filtered_data,filtered_data);
  kalman_filter_linear_acc->F.setIdentity();
  kalman_filter_linear_acc->H.setIdentity();
  kalman_filter_linear_acc->Q.setIdentity();
  kalman_filter_linear_acc->R.setIdentity();

  kalman_filter_linear_acc->R = kalman_filter_linear_acc->R*500; // sensor noise filtering  --> can be modified a external file.
}

void PoseEstimation::offset_init(Eigen::MatrixXd data,  int desired_sample_num)
{
  static int sample_num = 1;

  offset_data += data;

  if(sample_num == desired_sample_num)
  {
    offset_data = offset_data/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}


void PoseEstimation::estimation_processing(Eigen::MatrixXd data) // input entire force torque
{
 // tool_linear_acc_data = tool_linear_acc_data - offset_data;



  //filtered_data = kalman_filter_linear_acc->kalman_filtering_processing(tool_linear_acc_data);

  //calculate contact force
  contacted_force_torque(0,0) = data(0,0) -(mass_of_tool * tool_linear_acc_data(0,0))*-1;
  contacted_force_torque(1,0) = data(1,0) -(mass_of_tool * tool_linear_acc_data(1,0))*-1;
  contacted_force_torque(2,0) = data(2,0) -(mass_of_tool * tool_linear_acc_data(2,0))*-1;

  filtered_data = contacted_force_torque;

  //calculate inertia of tool
  //inertia_of_tool(0,0) = data(3,0)/tool_acc_data(3,0);
  //inertia_of_tool(1,1) = data(4,0)/tool_acc_data(4,0);
  //inertia_of_tool(2,2) = data(5,0)/tool_acc_data(5,0);


  //contacted_torque(0,0) = data(3,0) - inertia_of_tool(0,0)*tool_acc_data(3,0);
  //contacted_torque(1,0) = data(4,0) - inertia_of_tool(0,0)*tool_acc_data(4,0);
  //contacted_torque(2,0) = data(5,0) - inertia_of_tool(0,0)*tool_acc_data(5,0);
}

