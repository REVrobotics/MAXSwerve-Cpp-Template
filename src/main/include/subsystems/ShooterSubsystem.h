#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  // Subsystem methods go here.

  /**
   * Example: Do the thing
   */
  // void DoTheThing();

  // This method is called periodically by the CommandScheduler
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  // Left stick controls shooter "out" (forward Y) and "in" (backward Y)
  // double m_operatorController.GetLeftY(	)

  // Shooter motor
  CANSparkMax m_shooterMotor;
};