#include "camera/camera.h"

const int API_ID = cv::CAP_GSTREAMER;

Camera::Camera(int _width, int _height, CaptureMode _cm)
{
    width = _width;
    height = _height;
    cm = _cm;
}

Camera::~Camera()
{
    if (cap.isOpened())
    {
        cap.release();
    }
}

bool Camera::open()
{
    std::string gstream_str = get_gstream_str();
    return cap.open(gstream_str, API_ID);
}


bool Camera::get_frame(cv::Mat& frame)
{
    return cap.read(frame);
}

std::string Camera::get_gstream_str()
{
    std::stringstream result;
    result << "nvarguscamerasrc ! video/x-raw(memory:NVMM), ";
    result << "width=" << cm.width << ", ";
    result << "height=" << cm.height << ", ";
    result << "format=(string)NV12, ";
    result << "framerate=(fraction)" << cm.fps << "/1 ";
    result << "! nvvidconv ! video/x-raw, ";
    result << "width=(int)" << width << ", ";
    result << "height=(int)" << height << ", ";
    result << "format=(string)BGRx ! videoconvert ! appsink";

    return result.str();
}
