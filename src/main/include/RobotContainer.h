// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LEDSubsystem.h"
// #include "subsystems/ShooterSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
private:
 
  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // The operator's controller:
  // - Left stick up/down controls elevator
  frc2::CommandXboxController m_operatorController{OIConstants::kOperatorControllerPort};
  
  // Left bumper controls intake "in" - in while pressed?
  frc2::Trigger m_operatorLeftBumper = m_operatorController.LeftBumper();
  // Right bumper controls intoke "out" - out while pressed?
  frc2::Trigger m_operatorRightBumper = m_operatorController.RightBumper();
  // When set the robot goes at full throttle.  When clear full throttle is scaled down by
  // Should be k constants
  double button3_result;
  double throttle_percentage;
  bool fieldRelative;
  // The robot's subsystems and commands are defined here...
  // The robot's subsystems
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  LEDSubsystem m_led;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  // Timer
  frc::Timer timer0;
  
  void ConfigureButtonBindings();
};