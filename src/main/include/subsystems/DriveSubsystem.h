// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/imu/OnboardIMU.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include "wpi/math/kinematics/SwerveDriveOdometry.hpp"
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/array.hpp>

#include "Constants.h"
#include "MAXSwerveModule.h"

class DriveSubsystem : public wpi::cmd::SubsystemBase {
 public:

  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta velocities. Velocities range from [-1, 1]
   * and the linear velocities have no effect on the angular velocitiy.
   *
   * @param xSpeed        Velocity of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Velocity of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y velocities are relative to
   *                      the field.
   * @param period        The duration of the timestep the velocities should be applied
   *                      for.
   */
  void Drive(wpi::units::meters_per_second_t xVelocity,
             wpi::units::meters_per_second_t yVelocity,
             wpi::units::radians_per_second_t rot, bool fieldRelative,
             wpi::units::second_t period);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleVelocities(wpi::util::array<wpi::math::SwerveModuleVelocity, 4> desiredVelocities);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  wpi::units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  wpi::math::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(const wpi::math::Pose2d& pose);

  wpi::math::SwerveDriveKinematics<4> kDriveKinematics{
    m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation
  };

 private:
  wpi::math::Translation2d m_frontLeftLocation{+DriveConstants::kWheelBase / 2,
                         +DriveConstants::kTrackWidth / 2};
  wpi::math::Translation2d m_frontRightLocation{+DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2};
  wpi::math::Translation2d m_rearLeftLocation{-DriveConstants::kWheelBase / 2,
                         +DriveConstants::kTrackWidth / 2};
  wpi::math::Translation2d m_rearRightLocation{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  MAXSwerveModule m_frontLeft{DriveConstants::kFrontLeftBusId,
                  DriveConstants::kFrontLeftDrivingCanId,
                  DriveConstants::kFrontLeftTurningCanId,
                  DriveConstants::kFrontLeftChassisAngularOffset};
  MAXSwerveModule m_rearLeft{DriveConstants::kRearLeftBusId,
                  DriveConstants::kRearLeftDrivingCanId,
                  DriveConstants::kRearLeftTurningCanId,
                  DriveConstants::kRearLeftChassisAngularOffset};
  MAXSwerveModule m_frontRight{DriveConstants::kFrontRightBusId,
                  DriveConstants::kFrontRightDrivingCanId,
                  DriveConstants::kFrontRightTurningCanId,
                  DriveConstants::kFrontRightChassisAngularOffset};
  MAXSwerveModule m_rearRight{DriveConstants::kRearRightBusId,
                  DriveConstants::kRearRightDrivingCanId,
                  DriveConstants::kRearRightTurningCanId,
                  DriveConstants::kRearRightChassisAngularOffset};

  // The gyro sensor
  wpi::OnboardIMU m_gyro{wpi::OnboardIMU::MountOrientation::FLAT};

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  wpi::math::SwerveDriveOdometry<4> m_odometry{
      kDriveKinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()}};
};
