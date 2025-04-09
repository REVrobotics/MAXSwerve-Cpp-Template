// Declare a Subsystem to operate the 2025 intake elevator

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        // Constructor
        ElevatorSubsystem();
        
        // Destructor
        ~ElevatorSubsystem();

        // Actions to take regularly as the robot runs
        void Periodic() override;
        
        /***
         * Methods that directly do a thing
         ***/
        // Need method to raise/lower (move!) elevator (positive == up, negative == down)
        void setSpeed(double speed);
        void runForTime(units::second_t seconds, double speed);
        void autoRaise();

        /***
         * Methods that return a CommandPtr to a thing that does the thing
         ***/
        frc2::CommandPtr runSetSpeed(double speed);

        // Possible future methods to move to preset positions: trough, first coral, 2nd coral, receive-from-human-player

        // Need a Timer to tell us how long to run the Elevator
        frc::Timer m_elevatorTimer{};

    private:
        /***
         * Internal data to the subsystem
         */
        const int kElevatorLeftMotorCANId {10};
        const int kElevatorRightMotorCANId {11};
        const int kUpperLimitSwitchChannel {9};
        const int kLowerLimitSwitchChannel {8};
        
        /* Need 2 rev::Spark::SparkMax motors.
          These will need to be set so that one follows the other in reverse */
        rev::spark::SparkMax m_elevatorRaiseLowerMotor {kElevatorLeftMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        /* rev::spark::SparkMax m_elevatorRaiseLowerFollowerMotor{kElevatorRightMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        */
        // Need one or two Limit switches to tell us when the elevator needs to stop at the top/bottom
        frc::DigitalInput m_upperLimitSwitch {kUpperLimitSwitchChannel};
        frc::DigitalInput m_lowerLimitSwitch {kLowerLimitSwitchChannel};
        
        units::second_t m_raiseTime{1.25};
};