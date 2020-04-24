/**********************************

    Created on : 24th April 2020 
    Author     : Krishna Sandeep

**********************************/

#include <ros/ros.h>

#include "ukf/ukf.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ukf_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    UKF ukf(node, private_nh); //instance of UKF class

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce(); //invokes callback
        ukf.estimateVehicleState(); //runs the UKF estimation
        loop_rate.sleep();
    }

    return 0;
}