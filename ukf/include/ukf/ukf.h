/**********************************

    Created on : 24th April 2020 
    Author     : Krishna Sandeep

**********************************/

#ifndef UKF_UKF_H
#define UKF_UKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <utility>

#include <Eigen/Dense>

using namespace std; 
using namespace Eigen;

/*
--------- Assuming 2D motion, Vehicle state {x,y,theta,vx,vy} i.e., {position x, position y, heading angle, velocity x, velocity y}
*/

typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;

/* state consists of 5 elements [x, y, theta, vx, vy] in order and a 5x5 covariance matrix
   (x,y)   : position in map frame
   theta   : heading angle (ENU convention)
   (vx,vy) : velocity in base_link frame
*/
class VehicleState
{
    public:
        VehicleState();
        ~VehicleState(){}

        Vector5d vec;
        Matrix5d cov_mat; //covariance matrix
};

class MMInput
{
    public:
        MMInput();
        ~MMInput(){}

        double ax; //acceleration
        double ay;
        double omega; //yaw rate
        double delT; //time interval
};

class UKF
{
    public:
        UKF(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~UKF(){}

        void estimateVehicleState(); //runs the UKF estimation
    
    private:
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); //IMU callback
        void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& fix); //GNSS callback

        void initVehicleState(const double yaw); //Initializes the Vehicle state to default values
        void predict(); //the prediction step
        Vector5d applyMotionModel(Vector5d vec); //applies the model model
        void correct(const double x, const double y); //the correction step
        void publishVehicleState(); //publishes the vehicle state as a nav_msgs::Odometry message

        ros::Subscriber gnss_sub; //GNSS subscriber
        ros::Subscriber imu_sub; //IMU data subscriber
        ros::Publisher odom_pub; //publishes nav_msgs::Odometry msgs
        string _imu_topic, _gnss_topic, _odom_topic; //topic names to subscribe/publish
        double _noise_var_pos, _noise_var_yaw, _noise_var_vel, _noise_var_meas; //noise variances for position, yaw, velocity and measurement
        bool _init_state_provided; //True if initial state is provided
        double _init_x, _init_y, _init_yaw, _init_velx, _init_vely; //initial values for state
        double _init_var_pos, _init_var_yaw, _init_var_vel; //initial variances for position, yaw, velocity

        VehicleState _curr_state, _predicted_state;
        pair<double, double> _init_utm, _curr_utm; //Initial and current UTM reading in (easting, northing)
        pair<double, double> _vp_at_first_gnss; //vehicle position when the 1st GNSS reading is recieved
        bool _is_first_imu, _is_first_gnss; //boolean to verify if this is the first IMU, GNSS reading respectively
        bool _imu_available, _gnss_available; //true if a new imu/gnss reading is available
        double _prev_imu_time, _curr_imu_time; //Timestamps of two consecutive IMU readings
        MMInput _mmi; //current motion model input
        Matrix<double, 5, 11> _propagated_sigma_pt_mat; //matrix to store the sigma points propagated using the motion model
        Matrix5d _process_noise_mat; //Covariance matrix for the process noise
        Matrix2d _meas_noise_mat; //Covariance matrix for the measurement noise
};

#endif