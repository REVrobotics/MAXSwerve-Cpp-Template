#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  // Subsystem method declarations go here.

  // Constructor
  IntakeSubsystem();

  // This method is called periodically by the CommandScheduler
  void Periodic() override;

  // Move the intake to retracted or deployed position 
  //  (k_intakeDeployedPosition or k_intakeRetracted)
  void setIntakePosition(double position);

  // Deploy the intake
  frc2::CommandPtr deploy();

  // Retract the intake
  frc2::CommandPtr retract();

  // Start intake rollers in the "in" direction
  frc2::CommandPtr rollIn();

  // Start intake rollers in the "out" direction
  frc2::CommandPtr rollOut();

  // Stop intake rollers
  void stopRollers();
  
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
  
  rev::SparkRelativeEncoder m_intakeRaiseLowerEncoder = m_intakeRaiseLowerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 1);

  // Intake roller motor
  rev::CANSparkMax m_intakeRollerMotor{IntakeSubsystemConstants::kIntakeRollerCANId,
                                             rev::CANSparkLowLevel::MotorType::kBrushless};
                          
  // Deployed intake position value from encoder
  // TODO - determine correct value
  static constexpr double k_intakeDeployedPosition = 1.0;

  // Retracted intake position value from encoder
  // TODO - determine correct value
  static constexpr double k_intakeRetractedPosition = 0.0;

  static constexpr double k_rollerMotorSpeed = 0.25;

  bool m_rollerMotorOn = false;
  int m_rollerMotorDirection = -1; // -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)

};