#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

IntakeSubsystem::IntakeSubsystem(){};

void IntakeSubsystem::rollIn(){
  // Start / stop intake rollers in the "in" direction
  //
  // If left bumper is pressed once, activate intake "in" direction
  // If left bumper is pressed again, stop intake "in" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  if (m_rollerMotorOn == false) {
    m_rollerMotorDirection = -1;
    m_intakeRollerMotor.Set(m_rollerMotorDirection * k_rollerMotorSpeed);
    m_rollerMotorOn = true;
  } else {
    stopRollers();
  }
}


// Start intake rollers in the "out" direction
void IntakeSubsystem::rollOut(){
  // If right bumper is pressed once, activate intake "out" direction
  // If right bumper is pressed once, stop intake "out" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  if (m_rollerMotorOn == false) {
    m_rollerMotorDirection = 1;
    m_intakeRollerMotor.Set(m_rollerMotorDirection * k_rollerMotorSpeed);
    m_rollerMotorOn = true;
  } else {
    stopRollers();
  }
}

// Stop intake rollers
void IntakeSubsystem::stopRollers(){
  m_intakeRollerMotor.StopMotor();
  m_rollerMotorOn = false;
}

void IntakeSubsystem::Periodic(){}