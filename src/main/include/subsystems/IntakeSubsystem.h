#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  // Subsystem method declarations go here.

  /**
   * Example: Do the thing
   */
  // void DoTheThing();

  // This method is called periodically by the CommandScheduler
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  // Right stick controls intake raise (backward Y) and lower (forward Y)
  // double m_operatorController.GetLeftY(	)

   // Left bumper controls intake "in" - in while pressed?
  // frc2::Trigger m_operatorLeftBumper m_operatorController.LeftBumper();

  // Right bumper controls intoke "out" - out while pressed?
  // frc2::Trigger m_operatorRightBumper m_operatorController.RightBumper();

  // Intake raise/lower motor
  CANSparkMax m_intakeRaiseLowerMotor;

  // Intake roller motor
  CANSparkMax m_intakeRollerMotor;
};