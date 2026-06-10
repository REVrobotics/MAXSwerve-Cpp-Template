// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <wpi/commands2/InstantCommand.hpp>
#include <wpi/commands2/RunCommand.hpp>
// #include <wpi/commands2/SequentialCommandGroup.hpp>
#include <wpi/commands2/button/JoystickButton.hpp>
// #include <wpi/math/controller/PIDController.hpp>
// #include <wpi/math/controller/ProfiledPIDController.hpp>
// #include <wpi/math/geometry/Pose2d.hpp>
// #include <wpi/math/geometry/Translation2d.hpp>
// #include <wpi/math/trajectory/TrajectoryGenerator.hpp>
#include <wpi/math/util/MathUtil.hpp>

// #include <frc/shuffleboard/Shuffleboard.h>
// #include <frc/trajectory/Trajectory.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/SwerveControllerCommand.h>
// #include <units/angle.h>
// #include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer(wpi::units::second_t period)
    : m_period{period} {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(wpi::cmd::RunCommand(
      [this] {
        m_drive.Drive(
            -wpi::units::meters_per_second_t{wpi::math::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -wpi::units::meters_per_second_t{wpi::math::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -wpi::units::radians_per_second_t{wpi::math::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true,
            m_period);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  wpi::cmd::JoystickButton(&m_driverController,
                       wpi::NiDsXboxController::Button::kRightBumper)
      .WhileTrue(new wpi::cmd::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
}

wpi::cmd::CommandPtr RobotContainer::GetAutonomousCommand() {
  // no auto
  return wpi::cmd::InstantCommand(
          [this]() {
            m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, m_period);
          }).ToPtr();
}
