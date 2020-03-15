from enum import Enum


class Topics(Enum):
    # Topic names
    # Synchronize with cpp/topics.h

    # INPUT
    # Sensors
    CameraRaw = "/sensors/camera/raw"
    CameraDebug = "/sensors/camera/debug"

    BatteryStatus = "/sensors/battery/status"

    # Processed
    OpticFlow = "/processed/optic_flow"

    # OUTPUT
    # Actuators
    Display = "/actuators/oled"
    Motor = "/actuators/motor/raw"

    # Commands


class Parameters(Enum):
    # Parameter names
    # Synchronize with cpp/parameters.h
    CameraRawWidth = "/sensors/camera/raw/width"
    CameraRawHeight = "/sensors/camera/raw/height"
    CameraRawRate = "/sensors/camera/raw/rate"

    CameraDebugOn = "/sensors/camera/debug/on"
    CameraDebugWidth = "/sensors/camera/debug/width"
    CameraDebugHeight = "/sensors/camera/debug/height"
