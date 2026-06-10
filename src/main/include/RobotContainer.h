// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/Command.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/driverstation/NiDsXboxController.hpp>
#include <wpi/smartdashboard/SendableChooser.hpp>
#include <wpi/units/time.hpp>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer() = delete;

  /**
   * @param period Time period between calls to Periodic() functions;
   */
  explicit RobotContainer(wpi::units::second_t period);

  wpi::cmd::CommandPtr GetAutonomousCommand();

 private:
  // The driver's controller
  wpi::NiDsXboxController m_driverController{OIConstants::kDriverControllerPort};

  const wpi::units::second_t m_period;

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  // The chooser for the autonomous routines
  wpi::SendableChooser<wpi::cmd::Command*> m_chooser;

  void ConfigureButtonBindings();
};
