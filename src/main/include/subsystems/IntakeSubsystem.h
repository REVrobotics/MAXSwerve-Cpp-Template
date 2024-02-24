#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  // Subsystem method declarations go here.
  void IntakeSubsystem::setIntakePosition(double position);

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
  rev::CANSparkMax m_intakeRaiseLowerMotor{IntakeSubsystemConstants::kIntakeRaiseLowerCANId,
                                             rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_intakeRaiseLowerEncoder = m_intakeRaiseLowerMotor.GetEncoder();

  // Intake roller motor
  rev::CANSparkMax m_intakeRollerMotor{IntakeSubsystemConstants::kIntakeRollerCANId,
                                             rev::CANSparkLowLevel::MotorType::kBrushless};
                          
  // Deployed intake position value from encoder
  // TODO - determine correct value
  constexpr double k_intakeDeployedPosition = 1.0;

  // Retracted intake position value from encoder
  // TODO - determine correct value
  constexpr double k_intakeRetractedPosition = 0.0;

  bool m_rollerMotorOn = false;
  int m_rollerMotorDirection = -1; // -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)
};