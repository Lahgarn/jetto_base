#include <string>

// Synchronize with python enum Parameters in jetto_base/constants.py
namespace Parameters
{
    const std::string CameraRawWidth = "/sensors/camera/raw/width";
    const std::string CameraRawHeight = "/sensors/camera/raw/height";
    const std::string CameraRawRate = "/sensors/camera/raw/rate";

    const std::string CameraDebugOn = "/sensors/camera/debug/on";
    const std::string CameraDebugWidth = "/sensors/camera/debug/width";
    const std::string CameraDebugHeight = "/sensors/camera/debug/height";
}
