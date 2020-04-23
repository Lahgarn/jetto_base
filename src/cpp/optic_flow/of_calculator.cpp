#define HAVE_INLINE

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_multifit.h>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <sensor_msgs/Image.h>

#include "optic_flow/of_calculator.h"


OpticFlowCalculator::OpticFlowCalculator(std::string _images_topic, std::string _flow_topic, std::string _kp_topic)
	: it_(nh_)
{
    images_topic = _images_topic;
    flow_topic = _flow_topic;

    prev_frame = NULL;
    curr_frame = NULL;

    width = 960;
    height = 540;

    y_offset = 180;
    spacing = 40;
    flow_window = 20;

    flow_mat = get_flow_mat(flow_grid, width, height, spacing, y_offset);
    flow_obs = gsl_vector_alloc(flow_mat->size1);

    params = gsl_vector_alloc(2);
    cov = gsl_matrix_alloc(2, 2);
    chisq = (double*)malloc(sizeof(double));
    flws = gsl_multifit_linear_alloc(flow_mat->size1, flow_mat->size2);
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
        compute_flow();
    }
}

void OpticFlowCalculator::compute_flow()
{
    std::vector<uchar> features_found;
    std::vector<cv::Point2f> new_grid;

    // Compute LK flow
    cv::calcOpticalFlowPyrLK(
        prev_frame, curr_frame,         // Input images
        flow_grid, new_grid,            // Input/output points to track
        features_found,                 // Output vector whether a point was tracked
        cv::noArray(),                  // Output vector, lists errors (optional)
        cv::Size(flow_window, flow_window),  // Search window size
        3,                              // Maximum pyramid level to construct
        cv::TermCriteria(
            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
            20,                         // Maximum number of iterations
            0.3                         // Minimum change per iteration
        )
    );

    // Build observations
    build_flow(flow_obs, flow_grid, new_grid);

    // Solve
    gsl_multifit_linear(flow_mat, flow_obs, params, cov, chisq, flws);

    /* auto end_model = std::chrono::system_clock::now(); */

    float d_x = gsl_vector_get(params, 0);
    float d_theta = gsl_vector_get(params, 1);

    printf("Computed velocity: (%f, %f)\n", d_x, d_theta);
}


void OpticFlowCalculator::build_flow(std::vector<cv::Point2f> &new_grid)
{
    for(int i=0; i < static_cast<int>(grid_prev.size()); ++i)
    {
        cv::Point2f pp = flow_grid[i];
        cv::Point2f cp = new_grid[i];

        cv::Point2f flow = cp - pp;

        gsl_vector_set(flow_obs, 2 * i + 0, flow.x);
        gsl_vector_set(flow_obs, 2 * i + 1, flow.y);
    }
}
