# SDU-UR10E-Sensor-Common
  This is about sensor fitering and the signal process libraries for SDU-UR10E force torque sensor.

### Introduction ###
SDU-UR10E-Sensor-Common/sensor_filter includes the following filters available: 

* 1st-order Low Pass Filter 

* 1st-order High Pass Filter

* Kalman Filter

* Kalman Bucy Filter 

SDU-UR10E-Sensor-Common/ur10e_force_torque_sensor includes the following fuctions available:

* Signal Processing (Kalman filter)
* Offset Tunner
* A filter gain file load using YAML 
* Collision Detection
* Tool's pose, force and torque estimation (Future works) 

  Generally, we need to filter a sensor's value when we want to control a robot stably. These libraries offer for you to use basic sensor filters easily. In addition, the ur10e_force_torque_sensor library is suitable for ur10e F/T sensor to filter and process sensor signals. The library also includes various basic algorithms (for example, collision detection, tool's pose estimation) to do industrial tasks. The final point is that you can create filter/estimation algorithms and easily verify the algorithm by using these two libraries

### Dependencies ###
* [Eigen3] (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [YAML] (https://yaml.org/)
* [SDU-Math-Common] (https://github.com/RobotEmperor/SDU-Math-Common.git)

### Compatible Operating Systems ###
  Currently tested on:

* Ubuntu 18.04 (Bionic Beaver)

### Build ###
  Currently tested on:

* Cmake 

### Installation (Build Instruction)###

  sensor_filter

    git clone https://github.com/RobotEmperor/SDU-UR10E-Sensor-Common.git
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

### Classes and Functions ###

  sensor_filter

* LowPassFilter & HighPassFilter 

  Firstly, declare a new filter class. You have to set control_time and cutoff_frequency variables in these two filter classes. And then just call the following function.

For example(for LowPassFilter), 

    LowPassFilter lpf;
    lpf = new LowPassFilter;
    lpf->control_time = 0.002; //(500Hz)
    lpf->cutoff_frequency = 20; // (20Hz)
    
    //in the control loop 
    lpf->lpf_processing(input_data);
    
    //program ended
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
    kalman_filter.initialize(state_variables, measurement_variables); 
    
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
    
    //program ended
    delete kalman_filter;
    
    
