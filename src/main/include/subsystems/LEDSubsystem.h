#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/LEDPattern.h>
#include <frc/util/Color.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();
        void Periodic() override;
        
        frc2::CommandPtr RunPattern(frc::LEDPattern pattern);
        
        constexpr frc::Color ColorFlip(const frc::Color inputcolor);

    private:
        static constexpr int kPort = 9;
        static constexpr int kLength = 299;
        frc::AddressableLED m_led{kPort};
        std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
};