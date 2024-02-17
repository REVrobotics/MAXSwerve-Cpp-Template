#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem() {
  m_intakeRaiseLowerMotor = new CANSparkMax(IntakeSubsystemConstants::kIntakeRaiseLowerCANId);
  m_intakeRollerMotor = new CANSparkMax(IntakeSubsystemConstants::kIntakeRollerCANId);
}

void IntakeSubsystem::Periodic(){
    // If right stick Y axis is pressed back, raise intake
    // If right stick Y axis is pressed forward, lower intake

    // If left bumper is pressed, activate intake "in" direction

    // If right bumper is pressed, activate intake "out" direction 
}