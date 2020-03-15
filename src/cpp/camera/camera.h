#include <string>
#include <sstream>

#include <opencv2/videoio.hpp>


struct CaptureMode
{
    int width;
    int height;
    int fps;
};

const CaptureMode CAMERA_CM_1 = {3280, 2464, 21};
const CaptureMode CAMERA_CM_2 = {3280, 1848, 28};
const CaptureMode CAMERA_CM_3 = {1920, 1080, 30};
const CaptureMode CAMERA_CM_4 = {1280, 720, 60};
const CaptureMode CAMERA_CM_5 = {1280, 720, 120};


class Camera
{
private:
    // Attributes
    int width, height;
    CaptureMode cm;
    cv::VideoCapture cap;

    // Member functions
    std::string get_gstream_str();

public:
    Camera(int _width, int _height, CaptureMode _cm);
    ~Camera();

    bool open();
    bool get_frame(cv::Mat& frame);

};
