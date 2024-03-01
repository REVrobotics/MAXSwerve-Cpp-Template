// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // GetThrottle returns an analog value from -1 to 1. We need to transform that to a percentage
        button3_result = m_driverController.GetThrottle();
        // frc::SmartDashboard::PutNumber("Throttle", button3_result);
        // When the throttle is all the way towards the top the result is -1
        // The kids want this to be 100%.  So subtract one to get a value between
        // 0 and -2.  Then multiply by -1 to get rid of the negative and divide 
        // by 2 to get a value between 0 and 1.  This is our percentage.
        button3_result--;
        button3_result = button3_result * -1;
        // frc::SmartDashboard::PutNumber("Adjusted Throttle", button3_result);
        throttle_percentage = button3_result * 0.5;
        // Below a certain percentage the robot won't move at all.  Don't
        // let the throttle below this value.
        if (throttle_percentage < 0.15) {throttle_percentage = 0.15;}
        frc::SmartDashboard::PutNumber("Throttle percentage", throttle_percentage);
        
        // Pushing buttons 11 and 12 resets the Z axis heading.  This could
        // be useful if the gyro drifts a lot
        if (m_driverController.GetRawButton(11) && m_driverController.GetRawButton(12))
            { m_drive.ZeroHeading();}

        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetY() * throttle_percentage , OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetX()  * throttle_percentage, OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetTwist() * throttle_percentage, OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  // Start / stop intake rollers in the "in" direction
  m_operatorController.LeftBumper().OnTrue(new frc2::cmd::StartEnd(
    // Start rollers spinning in
    [this] { m_intake.rollIn(); },
    // Stop the rollers at the end of the command
    [this] { m_intake.stopRollers(); },
    // Requires the shooter subsystem
    {&m_intake}
  ));

  // Start / stop intake rollers in the "out" direction
  m_operatorController.RightBumper().OnTrue(new frc2::cmd::StartEnd(
    // Start rollers spinning out
    [this] { m_intake.rollOut(); },
    // Stop the rollers at the end of the command
    [this] { m_intake.stopRoller(); },
    // Requires the shooter subsystem
    {&m_intake}
  ));

  // If right stick Y axis is pressed forward, deploy intake
  m_rightStickForward().OnTrue(frc2::cmd::RunOnce([this]{ m_intake.deploy(); }));

  // If right stick Y axis is pressed backward, raise intake
  m_rightStickBackward().OnTrue(frc2::cmd::RunOnce([this]{ m_intake.retract(); }));
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},

      // WF- Keeping this example waypoint code in case we need to use something like
      // this in the future.  It is completely useless for now since we want to move in a 
      // straight line
      {frc::Translation2d{0.25_m, 0_m}, frc::Translation2d{0.75_m, 0_m}},
      
      // End 1 meter from where we started.  This is enough distance to exit the starting zone and 
      // earn some points
      frc::Pose2d{1_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
          {}));
}


