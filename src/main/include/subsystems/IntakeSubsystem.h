#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  // Subsystem method declarations go here.

  // Constructor
  IntakeSubsystem();

  // This method is called periodically by the CommandScheduler
  void Periodic() override;

  // Start intake rollers in the "in" direction
  void rollIn();

  // Start intake rollers in the "out" direction
  void rollOut();

  // Stop intake rollers
  void stopRollers();
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // WNF This appears to be the motor connected to the shooter??
  const int kIntakeRollerCANId = 9; // TODO - set real CANId

  // Intake roller motor
  rev::spark::SparkMax m_intakeRollerMotor{kIntakeRollerCANId,
                                             rev::spark::SparkLowLevel::MotorType::kBrushless};
                          
  static constexpr double k_rollerMotorSpeed = 0.25;

  bool m_rollerMotorOn = false;
  int m_rollerMotorDirection = -1; // -1 = IN, 1 = OUT, 0 = STOP (May need to flip IN and OUT)

};