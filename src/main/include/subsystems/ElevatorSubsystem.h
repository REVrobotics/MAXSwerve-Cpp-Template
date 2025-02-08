// Declase a Subsystem to operate the 2025 intake elevator

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        // Constructor
        ElevatorSubsystem();

        // Actions to take regularly as the robot runs
        void Periodic() override;
        
        /***
         * Methods that directly do a thing
         ***/
        // Need method to raise/lower (move!) elevator (positive == up, negative == down)
        void setSpeed(double speed);
        
        /***
         * Methods that return a CommandPtr to a thing that does the thing
         ***/
        // Possible future methods to move to preset positions: trough, first coral, 2nd coral, receive-from-human-player

    private:
        /***
         * Internal data to the subsystem
         */
        const int kElevatorLeftMotorCANId {99};  // TODO - set the real CANbus ID
        const int kElevatorRightMotorCANId {98};  // TODO - set the real CANbus ID
        
        /* Need 2 rev::Spark::SparkMax motors.
          These will need to be set so that one follows the other in reverse */
        rev::spark::SparkMax m_elevatorRaiseLowerMotor{kElevatorLeftMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_elevatorRaiseLowerFollowerMotor{kElevatorRightMotorCANId, 
                                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
        // Need one or two Limit switches to tell us when the elevator needs to stop at the top/bottom
        frc::DigitalInput m_topLimitSwitch {99};  // TODO - set the real digital input channel id
};