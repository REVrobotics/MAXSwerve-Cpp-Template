// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/MAXSwerveModule.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"

using namespace rev::spark;

MAXSwerveModule::MAXSwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingSpark(drivingCANId, SparkMax::MotorType::kBrushless),
      m_turningSpark(turningCANId, SparkMax::MotorType::kBrushless) {
  // Apply the respective configurations to the SPARKS. Reset parameters before
  // applying the configuration to bring the SPARK to a known good state.
  // Persist the settings to the SPARK to avoid losing them on a power cycle.
  m_drivingSpark.Configure(Configs::MAXSwerveModule::DrivingConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
  m_turningSpark.Configure(Configs::MAXSwerveModule::TurningConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);

  m_chassisAngularOffset = chassisAngularOffset;
  m_desiredState.angle =
      frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
  m_drivingEncoder.SetPosition(0);
}

frc::SwerveModuleState MAXSwerveModule::GetState() const {
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                          m_chassisAngularOffset}};
}

frc::SwerveModulePosition MAXSwerveModule::GetPosition() const {
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                          m_chassisAngularOffset}};
}

void MAXSwerveModule::SetDesiredState(
    const frc::SwerveModuleState& desiredState) {
  // Apply chassis angular offset to the desired state.
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle +
      frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

  // Optimize the reference state to avoid spinning further than 90 degrees.
  correctedDesiredState.Optimize(
      frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()}));

  m_drivingClosedLoopController.SetReference(
      (double)correctedDesiredState.speed, SparkMax::ControlType::kVelocity);
  m_turningClosedLoopController.SetReference(
      correctedDesiredState.angle.Radians().value(),
      SparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
}

void MAXSwerveModule::ResetEncoders() { m_drivingEncoder.SetPosition(0); }
