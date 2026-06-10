// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <rev/SparkMax.h>
#include <wpi/math/trajectory/TrapezoidProfile.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr wpi::units::meters_per_second_t kMaxVelocity = 4.8_mps;
constexpr wpi::units::radians_per_second_t kMaxAngularVelocity{2.0 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
// Distance between centers of right and left wheels on robot
constexpr wpi::units::meter_t kTrackWidth = 0.6731_m;
// Distance between centers of front and back wheels on robot
constexpr wpi::units::meter_t kWheelBase = 0.6731_m;

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2.0;
constexpr double kFrontRightChassisAngularOffset = 0.0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2.0;

// SPARK MAX CAN IDs
constexpr int kFrontLeftBusId = 0;
constexpr int kRearLeftBusId = 0;
constexpr int kFrontRightBusId = 0;
constexpr int kRearRightBusId = 0;

constexpr int kFrontLeftDrivingCanId = 11;
constexpr int kRearLeftDrivingCanId = 13;
constexpr int kFrontRightDrivingCanId = 15;
constexpr int kRearRightDrivingCanId = 17;

constexpr int kFrontLeftTurningCanId = 10;
constexpr int kRearLeftTurningCanId = 12;
constexpr int kFrontRightTurningCanId = 14;
constexpr int kRearRightTurningCanId = 16;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr wpi::units::meter_t kWheelDiameter = 0.0762_m;
constexpr wpi::units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxVelocity = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularVelocity = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

constexpr wpi::math::TrapezoidProfile<wpi::units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularVelocity, kMaxAngularAcceleration};
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants
