# SDU-UR10E-Sensor-Common
  This is about sensor fitering and the signal process libraries for SDU-UR10E force torque sensor.

## Introduction ##
SDU-UR10E-Sensor-Common/sensor_filter includes the following filters available: 

* 1st-order Low Pass Filter 

* 1st-order High Pass Filter

* Kalman Filter

* Kalman Bucy Filter 

SDU-UR10E-Sensor-Common/ur10e_force_torque_sensor includes the following functions available:

* Signal Processing (Kalman filter)
  * Adaptive gain algorithm (can change the kalman filter sensor noise gain R depending on the rate of changes of raw sensor data)
* Offset Tunner
* A filter gain file load using YAML 
* Collision Detection
* Tool's pose, force and torque estimation (Future works) 

  Generally, we need to filter a sensor's value when we want to control a robot stably. These libraries offer for you to use basic sensor filters easily. In addition, the ur10e_force_torque_sensor library is suitable for ur10e F/T sensor to filter and process sensor signals. The library also includes various basic algorithms (for example, collision detection, tool's pose estimation) to do industrial tasks. The final point is that you can create filter/estimation algorithms and easily verify the algorithm by using these two libraries

## Dependencies ##
* [Eigen3] (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [YAML] (https://yaml.org/)
* [SDU-Math-Common] (https://github.com/RobotEmperor/SDU-Math-Common.git)

## Compatible Operating Systems ##
  Currently tested on:

* Ubuntu 16.04 (Xenial Xerus)
* Ubuntu 18.04 (Bionic Beaver)

## Build ##
  Currently tested on:

* Cmake 

## Installation (Build Instruction) ##

  sensor_filter

    git clone https://github.com/RobotEmperor/SDU-UR10E-Sensor-Common.git
    cd SDU-UR10E-Sensor-Common
    cd sensor_filter
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install 

  ur10e_force_torque_sensor
    
    cd SDU-UR10E-Sensor-Common 
    cd ur10e_force_torque_sensor
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install 
    
  These two libraries are installed in /usr/local/lib and the headers are in /usr/local/include
  
## Cmake Example ## 

    cmake_minimum_required(VERSION 3.5)
    project(your_project)
    
    link_directories(/usr/local/lib)
    add_executable(your_project main.cpp)
    target_link_libraries(your_project ur10e_force_torque_sensor sdu_math)


## Classes and Functions ##

  ### sensor_filter ###

* LowPassFilter & HighPassFilter 

  Firstly, declare a new filter class. You have to set control_time and cutoff_frequency variables in these two filter classes. And then just call the following function.

For example(for LowPassFilter), 

    LowPassFilter lpf;
    lpf = new LowPassFilter;
    lpf->control_time = 0.002; //(500Hz)
    lpf->cutoff_frequency = 20; // (20Hz)
    
    //in the control loop 
    lpf->lpf_processing(input_data);// It returns double and you can use it. 
    
    //program is ended
    delete lpf;
    

* KalmanFilter & KalmanBucyFilter

  To use the class, you must design your system model in state space equation and then define matrix F,H,Q,R. And then define state_variables and measurement_variables in matrix form by using Eigen::MatrixXd. 
  
For example(for KalmanFilter), 

    Eigen::MatrixXd state_variables;
    Eigen::MatrixXd measurement_variables;
    
    state_variables.resize(6,1); // only force torque itself. 
    measurement_variables.resize(6,1);
    
    KalmanFilter kalman_filter;
    kalman_filter = new KalmanFilter
    kalman_filter->initialize(state_variables, measurement_variables); 
    
    //this example is about only force torque sensor itself, so F,H,Q set identity matrix.
    KalmanFilter->F.setIdentity();
    KalmanFilter->H.setIdentity();
    KalmanFilter->Q.setIdentity();
    
    //R is mesurement noise, you can desgin the gain.
    KalmanFilter->R.setIdentity() = KalmanFilter->R.setIdentity()*gain_R;
    
    // ** if you have specific system model, you have to design F H Q R variables in matrix form. 
    
    //in the control loop 
    KalmanFilter->kalman_filtering_processing(measurement_variables); // this function return to Eigen::MatrixXd
    // output filtered data in matrix form
    // in here, the output is force X, force Y, Force Z, Torque X, Torque Y, Torque Z in 6 X 1 matrix form.
    
    //program is ended
    delete kalman_filter;
    
  ### ur10e_force_torque_sensor ###

  Ur10eFTsensor
  
  This library is for using ur10e F/T sensor easily. It uses the sensor_filter library to filter raw force torque data. It also offers important data which can be used in control algorithm (for example, collision detection and pose estimation) 
  
  * parse_init_data(const std::string &path)
  
    To load sensor filter gains, you can insert the path of init_data.yaml.
  
  * initialize()
    
    Variables are initialized in this function. 
    
  * offset_init(Eigen::MatrixXd data, bool time_check)
  
    This function is to get offest value by sampling raw data of FT sensor for a certain period of time.
    
  * signal_processing(Eigen::MatrixXd data) 
  
    Filter algorithms are included.(Now only kalman filter)
    
  * collision_detection(Eigen::MatrixXd data)
  
    This function can detect collision from raw force torque data by using CUSUM method (it was included in sdu_math library). It returns int value 1 and -1 when collision is detected. (Value 0 is default and non-contact)
    
For example (How to use the library)

    Ur10eFTsensor ft_sensor;
    ft_sensor = new Ur10eFTsensor; 
    
    std::string init_data_path; // it is to load config file.
    init_data_path = "../config/init_data.yaml"; // it must be in your project.
    
    ft_sensor->parse_init_data(init_data_path);
    ft_sensor->initialize(); 
    
    //in the control loop
  
    ft_sensor->signal_processing(raw_force_torque_data); // output is stored in ft_sensor->ft_filtered_data
    
    // you can use this variables in your control algorithm "ft_sensor->ft_filtered_data"
    
    ft_sensor->collision_detection_processing(raw_force_torque_data); // this is to detect contact and collision 
    
    // 
    
    // program is ended 
    delete ft_sensor;

  
  
  
