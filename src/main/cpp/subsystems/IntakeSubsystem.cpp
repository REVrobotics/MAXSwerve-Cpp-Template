#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

IntakeSubsystem::IntakeSubsystem(){};

// Need to initialize encoder position to 0 on startup

// This needs to be a Command
void IntakeSubsystem::setIntakePosition(double position){
  # If encoder position has not met target rotations yet, drive motor
  static constexpr double k_intakeRaiseLowerMotorSpeed = 0.1;
  static constexpr double k_positionFudge = 0.05;

  double intakeRaiseLowerValue = m_intakeRaiseLowerEncoder.GetPosition();
  // If there's a change in direction, we need to at least stop, then reverse - 
  //   slow & reverse is even better 


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

void IntakeSubsystem::Periodic(){
  // If right stick Y axis is pressed back, raise intake
  // TODO - is backward negative or positive? Adjust > | < accordingly
  if (m_operatorController.getRightY() > 0)
  {
    IntakeSubsystem::setIntakePosition(k_intakeDeployedPosition);
  }
  // If right stick Y axis is pressed forward, lower intake
  if (m_operatorController.getRightY() < 0)
  {
    IntakeSubsystem::setIntakePosition(k_intakeRetractedPosition);
  }

  // If left bumper is pressed once, activate intake "in" direction
  // If left bumper is pressed again, stop intake "in" direction
  // If right bumper is pressed once, activate intake "out" direction
  // If right bumper is pressed once, stop intake "out" direction
  // REMEMBER: m_rollerMotorDirection : -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
 
  // If we're reversing direction, we need to slow down, stop, and speed up in reverse
  if (m_operatorController.GetLeftBumperPressed())
  {
    if (m_rollerMotorDirection != -1){
      m_intakeRollerMotor.StopMotor(); // Avoid sharp reverse in direction
      m_rollerMotorDirection = -1;
    }  
    m_rollerMotorOn = !m_rollerMotorOn; // Toggle motor on/off
  } else if (m_operatorController.GetRightBumperPressed)
  {
    if (m_rollerMotorDirection != 1){
      m_intakeRollerMotor.StopMotor(); // Avoid sharp reverse in direction
      m_rollerMotorDirection = 1;
    }  
    m_rollerMotorOn = !m_rollerMotorOn;
  }
  if (m_rollerMotorOn)
  {
    m_intakeRollerMotor.Set(m_rollerMotorDirection * k_rollerMotorSpeed);
  } else
  {
    m_intakeRollerMotor.StopMotor(); 
  }
}