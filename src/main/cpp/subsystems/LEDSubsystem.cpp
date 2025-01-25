#include <subsystems/LEDSubsystem.h>

LEDSubsystem::LEDSubsystem() {
    m_led.SetLength(kLength);
    m_led.Start();

    SetDefaultCommand(RunPattern(frc::LEDPattern::Solid(frc::Color::kBlack)).WithName("Off"));
}

void LEDSubsystem::Periodic() {
        m_led.SetData(m_ledBuffer);
}

frc2::CommandPtr LEDSubsystem::RunPattern(frc::LEDPattern pattern) {
    return Run([this, pattern = std::move(pattern)] { pattern.ApplyTo(m_ledBuffer); });
}

constexpr frc::Color LEDSubsystem::ColorFlip(const frc::Color inputcolor){
    return frc::Color(inputcolor.green, inputcolor.red, inputcolor.blue);
}