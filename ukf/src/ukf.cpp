/**********************************

    Created on : 24th April 2020 
    Author     : Krishna Sandeep

**********************************/

#include "ukf/ukf.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gps_common/conversions.h>

#include <cmath>

#include <Eigen/LU>

/* @brief Converts quaternion to euler */
double getYaw(geometry_msgs::Quaternion q_msg)
{
   tf2::Quaternion quat;
   tf2::fromMsg(q_msg, quat);

   tf2::Matrix3x3 mat(quat);
   double roll, pitch, yaw;
   mat.getRPY(roll, pitch, yaw);

   return yaw;
}

/* @brief Constructor */
VehicleState::VehicleState()
{
    vec = Vector5d::Zero();
    cov_mat = 1e-4*Matrix5d::Identity(); //small variances
}

/* @brief Constructor */
MMInput::MMInput()
{
    ax = 0.0;
    ay = 0.0;
    omega = 0.0;
    delT = 0.0;
}

/* @brief Constructor */
UKF::UKF(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters */
    private_nh.param("noise_var_pos", _noise_var_pos, 1e-4);
    ROS_INFO("noise_var_pos: %f", _noise_var_pos);
    private_nh.param("noise_var_yaw", _noise_var_yaw, 1e-4);
    ROS_INFO("noise_var_yaw: %f", _noise_var_yaw);
    private_nh.param("noise_var_vel", _noise_var_vel, 1e-4);
    ROS_INFO("noise_var_vel: %f", _noise_var_vel);
    private_nh.param("noise_var_meas", _noise_var_meas, 1e-4);
    ROS_INFO("noise_var_meas: %f", _noise_var_meas);
    private_nh.param("init_state_provided", _init_state_provided, false);
    ROS_INFO("init_state_provided: %d", _init_state_provided);
    private_nh.param("init_x", _init_x, 0.0);
    ROS_INFO("init_x: %f", _init_x);
    private_nh.param("init_y", _init_y, 0.0);
    ROS_INFO("init_y: %f", _init_y);
    private_nh.param("init_yaw", _init_yaw, 0.0);
    ROS_INFO("init_yaw: %f", _init_yaw);
    private_nh.param("init_velx", _init_velx, 0.0);
    ROS_INFO("init_velx: %f", _init_velx);
    private_nh.param("init_vely", _init_vely, 0.0);
    ROS_INFO("init_vely: %f", _init_vely);
    private_nh.param("init_var_pos", _init_var_pos, 1e-4);
    ROS_INFO("init_var_pos: %f", _init_var_pos);
    private_nh.param("init_var_yaw", _init_var_yaw, 1e-4);
    ROS_INFO("init_var_yaw: %f", _init_var_yaw);
    private_nh.param("init_var_vel", _init_var_vel, 1e-4);
    ROS_INFO("init_var_vel: %f", _init_var_vel);

    private_nh.param<std::string>("imu_topic", _imu_topic, "/imu/data");
    ROS_INFO("imu_topic: %s", _imu_topic.c_str());
    private_nh.param<std::string>("gnss_topic", _gnss_topic, "/fix");
    ROS_INFO("gnss_topic: %s", _gnss_topic.c_str());
    private_nh.param<std::string>("odom_topic", _odom_topic, "/odometry/car");
    ROS_INFO("odom_topic: %s", _odom_topic.c_str());

    imu_sub = node.subscribe(_imu_topic, 1, &UKF::imuCallback, this);
    gnss_sub = node.subscribe(_gnss_topic, 1, &UKF::gnssCallback, this);
    odom_pub = node.advertise<nav_msgs::Odometry>(_odom_topic, 1);

    //initializing values
    if(_init_state_provided)
    {
        _curr_state.vec(0) = _init_x;
        _curr_state.vec(1) = _init_y;
        _curr_state.vec(2) = _init_yaw;
        _curr_state.vec(3) = _init_velx;
        _curr_state.vec(4) = _init_vely;
        _curr_state.cov_mat.diagonal() << _init_var_pos, _init_var_pos, _init_var_yaw,
                                         _init_var_vel, _init_var_vel;
    }

    _process_noise_mat.diagonal() << _noise_var_pos, _noise_var_pos, _noise_var_yaw,
                                     _noise_var_vel, _noise_var_vel;

    _meas_noise_mat.diagonal() << _noise_var_meas, _noise_var_meas;

    _is_first_imu = true;
    _is_first_gnss = true;
    _imu_available = false;
    _gnss_available = false;

    _prev_imu_time = 0.0;
    _curr_imu_time = 0.0;

    _init_utm = make_pair(0.0, 0.0);
    _curr_utm = make_pair(0.0, 0.0);
    _vp_at_first_gnss = make_pair(0.0, 0.0);

    _propagated_sigma_pt_mat = Matrix<double, 5, 11>::Zero();

    //cout<<"Constructor done"<<endl;
}

/* @brief Callback for IMU data */
void UKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //cout<<"In imu callback"<<endl;

    if(_is_first_imu)
    {
        _prev_imu_time = msg->header.stamp.toSec();
        _curr_imu_time = msg->header.stamp.toSec();

        if(!_init_state_provided)
            initVehicleState(getYaw(msg->orientation));

        _is_first_imu = false; 
    }

    else
    {
        _imu_available =true;

        _prev_imu_time = _curr_imu_time;
        _curr_imu_time = msg->header.stamp.toSec();

        //fetching the motion model input
        _mmi.ax = msg->linear_acceleration.x;
        _mmi.ay = msg->linear_acceleration.y;
        _mmi.omega = msg->angular_velocity.z;
        _mmi.delT = _curr_imu_time - _prev_imu_time;
    }

    return;
}

/* @brief Callback for GNSS data */
void UKF::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& fix)
{
    //cout<<"In gnss callback"<<endl;
    
    std::string zone;
    
    if(_is_first_gnss)
    {
        //converting fix to UTM 
        //LLtoUTM arguments : (lat, long, northing, easting, zone)
        gps_common::LLtoUTM(fix->latitude, fix->longitude, _init_utm.second, _init_utm.first, zone);

        //saving position of the vehicle when 1st GNSS reading is received
        _vp_at_first_gnss.first = _curr_state.vec(0);
        _vp_at_first_gnss.second = _curr_state.vec(1);

        cout<<"Pos at 1st gnss, x : "<<_curr_state.vec(0)<<", y : "<<_curr_state.vec(1)<<endl;

        _is_first_gnss = false;
    }

    else
    {
        _gnss_available = true;
        //converting fix to UTM 
        //LLtoUTM arguments : (lat, long, northing, easting, zone)
        gps_common::LLtoUTM(fix->latitude, fix->longitude, _curr_utm.second, _curr_utm.first, zone);
    }

    return;
}

/* @brief Initializes the Vehicle state to default values i.e., (0.0, 0.0, heading, 0.0, 0.0)
where yaw is the initial yaw angle obtained from IMU when the program starts*/
void UKF::initVehicleState(const double yaw)
{
    cout<<"Initializing vehicle state with yaw = "<<yaw<<endl;
   
    _curr_state.vec(2) = yaw;

    return;
}

/* @brief Applying the motion model using acceleration and yaw rate data from IMU */
Vector5d UKF::applyMotionModel(Vector5d vec)
{
    //cout<<"Applying motion model"<<endl;
    
    Vector5d propagated_vec;

    double del_theta = _mmi.omega*_mmi.delT;
    Rotation2D<double> rot1(del_theta);
    Vector2d del_vec(_mmi.ax*_mmi.delT, _mmi.ay*_mmi.delT);
    propagated_vec(2) = vec(2) + del_theta; //updating yaw
    //bounding yaw between -PI and PI
    if(propagated_vec(2) > M_PI)
        propagated_vec(2) = propagated_vec(2) - 2*M_PI;
    else if(propagated_vec(2) <= -M_PI)
        propagated_vec(2) = propagated_vec(2) + 2*M_PI;

    /* Transforming velocities in previous base_link frame to current one and add acc*delT */
    propagated_vec.segment(3,2) = rot1.toRotationMatrix()*vec.segment(3,2) + del_vec;
    
    /* 
    This might look like using predicted velocity rather than velocity at previous time instance, but it is
    done to reduce computations, using v=u+at, x=ut+0.5at^2 is same as y=vt-0.5at^2
    */
    del_vec(0) = propagated_vec(3)*_mmi.delT - _mmi.ax*pow(_mmi.delT,2);
    del_vec(1) = propagated_vec(4)*_mmi.delT - _mmi.ay*pow(_mmi.delT,2);
    Rotation2D<double> rot2(propagated_vec(2));
    propagated_vec.segment(0,2) = rot2.inverse().toRotationMatrix()*del_vec + vec.segment(0,2);

    return propagated_vec;
}

/* @brief The prediction step */
void UKF::predict()
{
    //cout<<"In prediction step"<<endl;
    
    Matrix5d L = _curr_state.cov_mat.llt().matrixL(); //Cholesky factor L

    //calculating sigma points, using N+k = 3.0 for Gaussian pdfs
    _propagated_sigma_pt_mat.col(0) = applyMotionModel(_curr_state.vec);
    for(int i=1;i<=5;i++)
    {
        _propagated_sigma_pt_mat.col(i) = applyMotionModel(_curr_state.vec + sqrt(3.0)*L.col(i-1));
        _propagated_sigma_pt_mat.col(i+5) = applyMotionModel(_curr_state.vec - sqrt(3.0)*L.col(i-1));
    }

    //computing predicted mean
    _predicted_state.vec = (-2.0/3.0)*_propagated_sigma_pt_mat.col(0);
    for(int i=1;i<11;i++)
        _predicted_state.vec += (1.0/6.0)*_propagated_sigma_pt_mat.col(i);

    //computing predited covariance
    Matrix<double, 5, 11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;
    
    _predicted_state.cov_mat = (-2.0/3.0)*(del_state_mat.col(0))*(del_state_mat.col(0).transpose());
    for(int i=1;i<11;i++)
        _predicted_state.cov_mat += (1.0/6.0)*(del_state_mat.col(i))*(del_state_mat.col(i).transpose());
    
    _predicted_state.cov_mat += _process_noise_mat; //adding process noise covariance

    return;        
}

/* @brief The correction step */
void UKF::correct(const double x, const double y)
{
    //cout<<"In correction step"<<endl;
    
    Matrix<double,2,11> predicted_meas_mat = _propagated_sigma_pt_mat.block<2,11>(0,0);
    Vector2d predicted_meas_vec(_predicted_state.vec(0), _predicted_state.vec(1)); //since measurement is the position itself
    Matrix<double,2,11> del_meas_mat = predicted_meas_mat.colwise() - predicted_meas_vec;
    Matrix<double, 5, 11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;

    Matrix2d predicted_cov_mat;
    Matrix<double,5,2> cross_cov_mat; //cross covariance matrix

    predicted_cov_mat = (-2.0/3.0)*del_meas_mat.col(0)*(del_meas_mat.col(0).transpose());
    cross_cov_mat = (-2.0/3.0)*del_state_mat.col(0)*(del_meas_mat.col(0).transpose());
    for(int i=1;i<11;i++)
    {
        predicted_cov_mat += (1.0/6.0)*del_meas_mat.col(i)*(del_meas_mat.col(i).transpose());
        cross_cov_mat += (1.0/6.0)*del_state_mat.col(i)*(del_meas_mat.col(i).transpose());
    }
    
    predicted_cov_mat += _meas_noise_mat; //adding measurement noise covariance

    Matrix<double,5,2> kalman_gain = cross_cov_mat*predicted_cov_mat.inverse();

    Vector2d meas_vec(x,y); //actual measurements

    _curr_state.vec = _predicted_state.vec + kalman_gain*(meas_vec - predicted_meas_vec);
    _curr_state.cov_mat = _predicted_state.cov_mat - kalman_gain*predicted_cov_mat*(kalman_gain.transpose());

    return;
}

/* @brief Publishes the vehicle state as a nav_msgs::Odometry message*/
void UKF::publishVehicleState()
{
    //cout<<"Publishing vehicle state"<<endl;
    
    nav_msgs::Odometry vs_msg;
    vs_msg.header.stamp = ros::Time::now();
    vs_msg.header.frame_id = "map";
    vs_msg.child_frame_id = "base_link";

    vs_msg.pose.pose.position.x = _curr_state.vec(0);
    vs_msg.pose.pose.position.y = _curr_state.vec(1);
    vs_msg.pose.pose.position.z = 0.0;

    Eigen::AngleAxisd rot_mat (_curr_state.vec(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat(rot_mat); //rotation matrix stored as a quaternion
    vs_msg.pose.pose.orientation.x = quat.x();
    vs_msg.pose.pose.orientation.y = quat.y();
    vs_msg.pose.pose.orientation.z = quat.z();
    vs_msg.pose.pose.orientation.w = quat.w();

    vs_msg.twist.twist.linear.x = _curr_state.vec(3);
    vs_msg.twist.twist.linear.y = _curr_state.vec(4);
    vs_msg.twist.twist.linear.z = 0.0;
    vs_msg.twist.twist.angular.x = 0.0;
    vs_msg.twist.twist.angular.y = 0.0;
    vs_msg.twist.twist.angular.z = _mmi.omega; //this is not estimated using UKF, so IMU reading is directly used

    for(int i=0;i<36;i++)
    {
        vs_msg.pose.covariance[i] = 1e-6; //a very small number instead of zero
        vs_msg.twist.covariance[i] = 1e-6;
    }

    vs_msg.pose.covariance[0] = _curr_state.cov_mat(0,0); //variance of x
    vs_msg.pose.covariance[7] = _curr_state.cov_mat(1,1); //variance of y
    vs_msg.pose.covariance[35] = _curr_state.cov_mat(2,2); //variance of theta
    vs_msg.twist.covariance[0] = _curr_state.cov_mat(3,3); //variance of vx
    vs_msg.twist.covariance[7] = _curr_state.cov_mat(4,4); //variance of vy

    odom_pub.publish(vs_msg); //publishing odometry msg

    return;
}

/* @brief Runs the UKF estimation */
void UKF::estimateVehicleState()
{
    //cout<<"Estimating state"<<endl;
    
    if(_imu_available)
    {
        predict(); //the prediction step

        _imu_available = false; //false until new imu reading is received
    }
    
    if(_gnss_available)
    {
        double meas_x = (_curr_utm.first - _init_utm.first) + _vp_at_first_gnss.first;
        double meas_y = (_curr_utm.second - _init_utm.second) + _vp_at_first_gnss.second;

        //cout<<"meas x : "<<meas_x<<" , meas y : "<<meas_y<<endl;

        correct(meas_x, meas_y); //correction step

        _gnss_available = false; //false until new gnss reading is received
    }

    else
        _curr_state = _predicted_state; //since no new measurements

    publishVehicleState(); //publishing the state

    return;
}