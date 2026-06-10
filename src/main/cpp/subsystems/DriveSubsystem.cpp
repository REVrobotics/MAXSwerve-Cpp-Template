// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/hal/UsageReporting.hpp>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() {
  m_gyro.ResetYaw();                    
  // Usage reporting for MAXSwerve template
  static int instanceNum{0};
  HAL_ReportUsage("kResourceType_RobotDrive", instanceNum,
                  "kRobotDriveSwerve_MaxSwerve");
  ++instanceNum;                  
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(wpi::units::meters_per_second_t xVelocity,
                           wpi::units::meters_per_second_t yVelocity,
                           wpi::units::radians_per_second_t rot,
                           bool fieldRelative, wpi::units::second_t period) {
  // Convert the commanded speeds into the correct units for the drivetrain
  wpi::units::meters_per_second_t xVelocityDelivered =
      xVelocity.value() * DriveConstants::kMaxVelocity;
  wpi::units::meters_per_second_t yVelocityDelivered =
      yVelocity.value() * DriveConstants::kMaxVelocity;
  wpi::units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularVelocity;

  wpi::math::ChassisVelocities chassisVelocities{xVelocityDelivered, yVelocityDelivered, rotDelivered};
  if (fieldRelative) {
    chassisVelocities =
        chassisVelocities.ToRobotRelative(m_gyro.GetRotation2d());
  }
  chassisVelocities = chassisVelocities.Discretize(period);

  auto [fl, fr, rl, rr] = kDriveKinematics.DesaturateWheelVelocities(
      kDriveKinematics.ToSwerveModuleVelocities(chassisVelocities), DriveConstants::kMaxVelocity);
  m_frontLeft.SetDesiredVelocity(fl);
  m_frontRight.SetDesiredVelocity(fr);
  m_rearLeft.SetDesiredVelocity(rl);
  m_rearRight.SetDesiredVelocity(rr);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredVelocity(
      wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d{45_deg}});
  m_frontRight.SetDesiredVelocity(
      wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredVelocity(
      wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredVelocity(
      wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleVelocities(
    wpi::util::array<wpi::math::SwerveModuleVelocity, 4> desiredVelocities) {
  auto [fl, fr, rl, rr] = kDriveKinematics.DesaturateWheelVelocities(
    desiredVelocities, DriveConstants::kMaxVelocity);
  m_frontLeft.SetDesiredVelocity(fl);
  m_frontRight.SetDesiredVelocity(fr);
  m_rearLeft.SetDesiredVelocity(rl);
  m_rearRight.SetDesiredVelocity(rr);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

wpi::units::degree_t DriveSubsystem::GetHeading() {
  return wpi::math::Rotation2d(m_gyro.GetAngleZ()).Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.ResetYaw(); }

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetAccelZ().value();
}

wpi::math::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(const wpi::math::Pose2d& pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
