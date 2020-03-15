#include <opencv2/features2d.hpp>

#include <sensor_msgs/Image.h>

#include "jetto_base/Point2D.h"
#include "jetto_base/Keypoints.h"
#include "optic_flow/of_calculator.h"


OpticFlowCalculator::OpticFlowCalculator(std::string _images_topic, std::string _flow_topic, std::string _kp_topic)
	: it_(nh_)
{
    images_topic = _images_topic;
    flow_topic = _flow_topic;

    prev_frame = NULL;
    curr_frame = NULL;

    keypoint_pub_ = nh_.advertise<jetto_base::Keypoints>(_kp_topic, 10);
}

OpticFlowCalculator::~OpticFlowCalculator()
{

}

void OpticFlowCalculator::start()
{
    image_sub_ = it_.subscribe(images_topic, 2, &OpticFlowCalculator::on_receive_image, this);

    ROS_INFO("Started listening in %s", images_topic.c_str());
}

void OpticFlowCalculator::on_receive_image(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image");

    prev_frame = curr_frame;
    try
    {
        curr_frame = cv_bridge::toCvCopy(msg, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(curr_frame && prev_frame)
    {
        cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints;

		ROS_INFO("Starting ORB");
		detector->detect(curr_frame->image, keypoints);
		ROS_INFO("ORB found %d points", (int)keypoints.size());

        std_msgs::Header header;
        header.seq = msg->header.seq;
        header.stamp = ros::Time::now();
        header.frame_id = msg->header.frame_id;

        jetto_base::Keypoints kps;
        kps.header = header;
        kps.img_seq = msg->header.seq;
        for(int i=0; i<keypoints.size(); i++)
        {
            jetto_base::Point2D point;
            point.x = keypoints[i].pt.x;
            point.y = keypoints[i].pt.y;
            kps.keypoints.push_back(point);
        }

        keypoint_pub_.publish(kps);


		/* cv::Mat flowxy; */
		/* ROS_INFO("Starting OF"); */
        /* cv::calcOpticalFlowFarneback(prev_frame->image, curr_frame->image, flowxy, */
		/* 							 0.5, 3, 15, 3, 5, 1.2, 0); */
		/* ROS_INFO("Ended OF"); */
    }
}
