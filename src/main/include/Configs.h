#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .OutputRange(-1, 1);

    return drivingConfig;
  }

  static SparkMaxConfig& TurningConfig() {
    static SparkMaxConfig turningConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    turningConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    turningConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(true)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    turningConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return turningConfig;
  }
};
}  // namespace Configs
