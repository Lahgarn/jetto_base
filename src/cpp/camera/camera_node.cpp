#include <string>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "camera/camera.h"
#include "parameters.h"
#include "topics.h"


// Default parameters
const int DEFAULT_RAW_WIDTH = 3280;
const int DEFAULT_RAW_HEIGHT = 2460;
const int DEFAULT_RAW_RATE = 10;

const int DEFAULT_DEBUG_WIDTH = 328;
const int DEFAULT_DEBUG_HEIGHT = 246;
const bool DEFAULT_DEBUG_ON = false;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera");

    ros::NodeHandle n;

    // Parameters
    int raw_width, raw_height, raw_rate;

    n.param(Parameters::CameraRawWidth, raw_width, DEFAULT_RAW_WIDTH);
    n.param(Parameters::CameraRawHeight, raw_height, DEFAULT_RAW_HEIGHT);
    n.param(Parameters::CameraRawRate, raw_rate, DEFAULT_RAW_RATE);

    // Prompt parameters
    ROS_INFO("Camera parameters");
    ROS_INFO("width: %d, height: %d, rate: %d", raw_width, raw_height, raw_rate);

    // Publisher setup
    image_transport::ImageTransport it(n);
    image_transport::Publisher raw_pub = it.advertise(Topics::CameraRaw, 1);

    // Camera setup
    Camera cam(raw_width, raw_height, CAMERA_CM_1);
    ros::Rate loop_rate(raw_rate);

    if(!cam.open())
    {
        ROS_ERROR("Couldn't open the camera!");
        return -1;
    }

    cv::Mat frame;
    int count = 0;
    while(ros::ok())
    {
        cam.get_frame(frame);
        if(frame.empty())
        {
            ROS_ERROR("Received an empty frame!");
            return -1;
        }

        // Create the header
        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();
        header.frame_id = "camera";

        // Build the whole message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_pub.publish(msg);

        ROS_INFO("Image sent");

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
