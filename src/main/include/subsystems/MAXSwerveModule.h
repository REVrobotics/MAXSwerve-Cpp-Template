// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

using namespace rev::spark;

class MAXSwerveModule {
 public:
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  MAXSwerveModule(int busCANId, int driveCANId, int turningCANId,
                  double chassisAngularOffset);

  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  wpi::math::SwerveModuleVelocity GetVelocity() const;

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  wpi::math::SwerveModulePosition GetPosition() const;

  /**
   * Sets the desired velocity for the module.
   *
   * @param desiredVelocity Desired velocity with velocity and angle.
   */
  void SetDesiredVelocity(const wpi::math::SwerveModuleVelocity& desiredVelocity);

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

 private:
  SparkMax m_drivingSpark;
  SparkMax m_turningSpark;

  SparkRelativeEncoder m_drivingEncoder = m_drivingSpark.GetEncoder();
  SparkAbsoluteEncoder m_turningAbsoluteEncoder =
      m_turningSpark.GetAbsoluteEncoder();

  SparkClosedLoopController m_drivingClosedLoopController =
      m_drivingSpark.GetClosedLoopController();
  SparkClosedLoopController m_turningClosedLoopController =
      m_turningSpark.GetClosedLoopController();

  double m_chassisAngularOffset = 0;
  wpi::math::SwerveModuleVelocity m_desiredVelocity{0_mps, wpi::math::Rotation2d()};
};
