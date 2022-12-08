// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

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
/* Driving Parameters */
constexpr units::meters_per_second_t kMaxSpeed = 4.4_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

/* Chassis configuration */
constexpr units::meter_t kTrackWidth =
    0.6731_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.6731_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

/* SPARK MAX CAN IDs */
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
constexpr bool kTurningEncoderInverted = true;

constexpr double kDrivingMotorReduction =
    66.0 / 13.0;  // rotations of motor / rotations of output
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    (double)kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) /
     (double)kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = 0.22;
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 4.4_mps;
constexpr auto kMaxAcceleration = 4.4_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
