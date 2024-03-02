#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>


#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

IntakeSubsystem::IntakeSubsystem(){};

// Need to initialize encoder position to 0 on startup

// This needs to be a Command
void IntakeSubsystem::setIntakePosition(double position){
  // If encoder position has not met target rotations yet, drive motor
  static constexpr double k_intakeRaiseLowerMotorSpeed = 0.1;
  static constexpr double k_positionFudge = 0.05;

  double intakeRaiseLowerValue = m_intakeRaiseLowerEncoder.GetPosition();

  if(intakeRaiseLowerValue < (position - k_positionFudge))    // If the position is less than requested, move forward
  {
    m_intakeRaiseLowerMotor.Set(k_intakeRaiseLowerMotorSpeed);
  } else if (intakeRaiseLowerValue > (position + k_positionFudge))  // If the position is above requested, move backward
  {
    m_intakeRaiseLowerMotor.Set(-k_intakeRaiseLowerMotorSpeed);
  } else  // Position is about right, stop
  {
    m_intakeRaiseLowerMotor.StopMotor();
  }
}

// Start / stop intake rollers in the "in" direction
//
// If left bumper is pressed once, activate intake "in" direction
// If left bumper is pressed again, stop intake "in" direction
// REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
// If we're reversing direction, we need to slow down, stop, and speed up in reverse
frc2::CommandPtr IntakeSubsystem::rollIn(){
  return this->StartEnd(
    // Start rollers spinning in
    [this] { 
        m_rollerMotorDirection = -1;
        m_intakeRollerMotor.Set(m_rollerMotorDirection * k_rollerMotorSpeed); 
        },
    // Stop the rollers at the end of the command
    [this] { m_intakeRollerMotor.stopMotor(); },
    // Requires the shooter subsystem
    {&m_intake}
  );
}


// Start intake rollers in the "out" direction
void IntakeSubsystem::rollOut(){
  // If right bumper is pressed once, activate intake "out" direction
  // If right bumper is pressed once, stop intake "out" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  return this->StartEnd(
    // Start rollers spinning in
    [this] { 
        m_rollerMotorDirection = 1;
        m_intakeRollerMotor.Set(m_rollerMotorDirection * k_rollerMotorSpeed); 
        },
    // Stop the rollers at the end of the command
    [this] { m_intakeRollerMotor.stopMotor(); },
    // Requires the shooter subsystem
    {&m_intake}
  ); 
}

// Stop intake rollers
void IntakeSubsystem::stopRollers(){
  m_intakeRollerMotor.StopMotor();
}

// Deploy the intake
frc2::CommandPtr IntakeSubsystem::deploy(){
    return this->RunOnce([this] { IntakeSubsystem::setIntakePosition(k_intakeDeployedPosition);});
}

// Retract the intake
frc2::CommandPtr IntakeSubsystem::retract(){
  return this->RunOnce([this] { IntakeSubsystem::setIntakePosition(k_intakeRetractedPosition);});
}

void IntakeSubsystem::Periodic(){}