#include <string>

// Synchronize with python enum Topics in jetto_base/constants.py
namespace Topics
{
    const std::string CameraRaw = "/sensors/camera";

    const std::string ORBKeypoints = "/processed/orb_keypoints";
    const std::string OpticFlow = "/processed/optic_flow";
}
