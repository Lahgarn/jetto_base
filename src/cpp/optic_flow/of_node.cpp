#include <ros/ros.h>

#include "optic_flow/of_calculator.h"
#include "topics.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "optical_flow");
    ros::NodeHandle n;

    ROS_INFO("Started optic flow node!");

    OpticFlowCalculator ofc(Topics::CameraRaw, Topics::OpticFlow, Topics::ORBKeypoints);

    ofc.start();

    ros::spin();

    return 0;
}
