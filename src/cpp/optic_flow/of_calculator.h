#include <string>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class OpticFlowCalculator
{
private:
    // Attributes
    std::string     images_topic;   // Input topic
    std::string     flow_topic;     // Output topic

    cv_bridge::CvImagePtr   prev_frame;    // Previous image to compute the optic flow
    cv_bridge::CvImagePtr   curr_frame;    // Current image to compute the optic flow

    int width, height;

    // Flow model parameters
    int spacing, y_offset, flow_window;
    std::vector<cv::Point2f> flow_grid;

    gsl_matrix *flow_mat;
    gsl_vector *flow_obs;

    // Least squares
    gsl_vector *params;
    gsl_matrix *cov;
    double *chisq;
    gsl_multifit_linear_workspace *flws;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;


    // Member functions
    void on_receive_image(const sensor_msgs::ImageConstPtr& msg);
    void compute_flow();
    void build_flow(std::vector<cv::Point2f> &new_grid);


public:
    OpticFlowCalculator(std::string _images_topic, std::string _flow_topic, std::string _kp_topic);
    ~OpticFlowCalculator();

    void start();
};
