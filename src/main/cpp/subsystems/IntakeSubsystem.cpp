#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

IntakeSubsystem::IntakeSubsystem(){};

// Need to initialize encode position to 0 on startup

void IntakeSubsystem::setIntakePosition(double position){
      # If encoder position has not met target rotations yet, drive motor
      double k_intakeRaiseLowerMotorSpeed = 0.1;
      double k_positionFudge = 0.05;

      double intakeRaiseLowerValue = m_intakeRaiseLowerEncoder.GetPosition();
      if(intakeRaiseLowerValue < (position - k_positionFudge))
      {
        m_intakeRaiseLowerMotor.Set(k_intakeRaiseLowerMotorSpeed);
      } else if (intakeRaiseLowerValue > (position + k_positionFudge))
      {
        m_intakeRaiseLowerMotor.Set(-k_intakeRaiseLowerMotorSpeed);
      } else
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

  // If left bumper is pressed, activate intake "in" direction
  // If right bumper is pressed, activate intake "out" direction
  double k_rollerMotorSpeed = 0.25;
  bool motorOn = false;
  if (m_operatorController.GetLeftBumperPressed())
  {
    // Set direction "in"
    if (motorOn == false)
    {
      motorOn = true;
      // Set intake motor to reverse speed
      m_intakeRollerMotor.Set(-k_rollerMotorSpeed);
    } else {
      motorOn = false;
      m_intakeRollerMotor.StopMotor();
    }
  } else if (m_operatorController.GetRightBumperPressed)
  {
    // Set direction "out"
    if(motorOn == false)
    {
      motorOn = true;
      // Set intake motor to forward speed
      m_intakeRollerMotor.Set(k_rollerMotorSpeed);
    } else
    {
      motorOn = false;
      m_intakeRollerMotor.StopMotor();
    }
  }
 

}