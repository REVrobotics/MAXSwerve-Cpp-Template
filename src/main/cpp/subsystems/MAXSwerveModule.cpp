// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/MAXSwerveModule.h"

#include "Configs.h"

using namespace rev::spark;

MAXSwerveModule::MAXSwerveModule(const int busCANId, const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingSpark(busCANId, drivingCANId, SparkMax::MotorType::kBrushless),
      m_turningSpark(busCANId, turningCANId, SparkMax::MotorType::kBrushless) {
  // Apply the respective configurations to the SPARKS. Reset parameters before
  // applying the configuration to bring the SPARK to a known good state.
  // Persist the settings to the SPARK to avoid losing them on a power cycle.
  m_drivingSpark.Configure(Configs::MAXSwerveModule::DrivingConfig(),
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
  m_turningSpark.Configure(Configs::MAXSwerveModule::TurningConfig(),
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);

  m_chassisAngularOffset = chassisAngularOffset;
  m_desiredVelocity.angle =
      wpi::math::Rotation2d(wpi::units::radian_t{m_turningAbsoluteEncoder.GetPosition().Get()});
  m_drivingEncoder.SetPosition(0);
}

wpi::math::SwerveModuleVelocity MAXSwerveModule::GetVelocity() const {
  return {wpi::units::meters_per_second_t{m_drivingEncoder.GetVelocity().Get()},
          wpi::units::radian_t{m_turningAbsoluteEncoder.GetPosition().Get() -
                          m_chassisAngularOffset}};
}

wpi::math::SwerveModulePosition MAXSwerveModule::GetPosition() const {
  return {wpi::units::meter_t{m_drivingEncoder.GetPosition().Get()},
          wpi::units::radian_t{m_turningAbsoluteEncoder.GetPosition().Get() -
                          m_chassisAngularOffset}};
}

void MAXSwerveModule::SetDesiredVelocity(
    const wpi::math::SwerveModuleVelocity& desiredVelocity) {
  // Apply chassis angular offset to the desired state.
  wpi::math::SwerveModuleVelocity correctedDesiredVelocity{};
  correctedDesiredVelocity.velocity = desiredVelocity.velocity;
  correctedDesiredVelocity.angle =
      desiredVelocity.angle +
      wpi::math::Rotation2d(wpi::units::radian_t{m_chassisAngularOffset});

  wpi::math::Rotation2d encoderRotation{
      wpi::units::radian_t{m_turningAbsoluteEncoder.GetPosition().Get()}};

  // Optimize the desired velocity to avoid spinning further than 90 degrees,
  // then scale velocity by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  auto velocity =
      correctedDesiredVelocity.Optimize(encoderRotation).CosineScale(encoderRotation);
      
  m_drivingClosedLoopController.SetSetpoint((double)velocity.velocity,
                                            SparkMax::ControlType::kVelocity);
  m_turningClosedLoopController.SetSetpoint(
      velocity.angle.Radians().value(),
      SparkMax::ControlType::kPosition);

  m_desiredVelocity = desiredVelocity;
}

void MAXSwerveModule::ResetEncoders() { m_drivingEncoder.SetPosition(0); }
