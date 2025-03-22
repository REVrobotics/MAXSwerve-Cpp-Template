// Copyright (cterOfRotation) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     //-m_gyro.getAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                     units::degree_t{-navx.GetAngle()}}), //-navx
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  //HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
  //           HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}

void DriveSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("MXP Gyro Angle", -navx.GetAngle());
  // frc::SmartDashboard::PutNumber("ADIS16470 Gyro Angle",  -m_gyro.getAngle(frc::ADIS16470_IMU::IMUAxis::kZ).value());
  
  frc::SmartDashboard::PutNumber("MXP Gyro Turn Rate", navx.GetRate());
  // frc::SmartDashboard::PutNumber("ADIS16470 Gyro Turn Rate", m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value());

  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::radian_t{
                       // -m_gyro.getAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                       units::degree_t{-navx.GetAngle()}}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  // fieldRelative is hardwired to false at the moment
  auto states = kDriveKinematics.ToSwerveModuleStates(
      false
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    //-m_gyro.getAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
                    units::degree_t{-navx.GetAngle()}}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});
          
  frc::SmartDashboard::PutNumber("DriveTrain X Speed Delivered", xSpeedDelivered);
  frc::SmartDashboard::PutNumber("DriveTrain Y Speed Delivered", ySpeedDelivered);
  frc::SmartDashboard::PutNumber("DriveTrain Rotation Delivered", rotDelivered);
  // frc::SmartDashboard::PutNumber("Navx Angle", navx.GetAngle());
  // frc::SmartDashboard::PutNumber("Swerve Field Relative", fieldRelative);
  
  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {

  return frc::Rotation2d(
            // units::radian_t{-m_gyro.getAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}).Degrees();
            units::radian_t{units::degree_t{-navx.GetAngle()}}).Degrees();
          
}
      
       
      

// void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }
void DriveSubsystem::ZeroHeading() { navx.Reset(); }

double DriveSubsystem::GetTurnRate() {
  //return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
  return navx.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
