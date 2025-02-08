#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include "RobotContainer.h"

// Define our ElevatorSubsystem Constructor
ElevatorSubsystem::ElevatorSubsystem(){
        // Nothing to do here right now
};

// Set the Elevator motor speeds to raise or lower
void ElevatorSubsystem::setSpeed(double speed){
        if (speed > 0){
                if (m_topLimitSwitch.Get() == true){
                        m_elevatorRaiseLowerMotor.Set(0);
                } else {
                        m_elevatorRaiseLowerMotor.Set(speed);
                }
        } else { 
                m_elevatorRaiseLowerMotor.Set(speed);
        }
};

// Package the setSpeed method for use as a Command
frc2::CommandPtr ElevatorSubsytem::RunSetSpeed(double speed) {
    return this->Run(
        [this, speed] { this->setSpeed(speed); });
}