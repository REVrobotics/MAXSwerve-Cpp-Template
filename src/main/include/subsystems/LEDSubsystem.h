#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/LEDPattern.h>
#include <frc/util/Color.h>

#pragma once

frc::Color ColorFlip(frc::Color inputcolor);

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();
        void Periodic() override;
        void ApplyPattern(frc::LEDPattern pattern);
        frc2::CommandPtr RunPattern(frc::LEDPattern pattern);

    private:
        static constexpr int kPort = 9;
        static constexpr int kLength = 300;
        frc::AddressableLED m_led{kPort};
        std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
};