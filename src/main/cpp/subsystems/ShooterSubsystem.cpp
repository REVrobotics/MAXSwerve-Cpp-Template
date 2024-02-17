#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

ShooterSubsystem::ShooterSubsystem() {
  m_shooterMotor = new CANSparkMax(ShooterSubsystemConstants::kShooterCANId);
}

void ShooterSubsystem::Periodic(){
    // If left stick Y axis is pressed forward, drive the shooter motor outwards 
    // If left stick Y axis is pressed backward, drive the shooter motor inwards

}

