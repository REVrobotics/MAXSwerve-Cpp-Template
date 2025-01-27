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
    return this->Run(
        [this, pattern = std::move(pattern)] { pattern.ApplyTo(m_ledBuffer); });
}

// Our LED strip has the red and green wires flipped, so swap the R and G values in the color
frc::Color ColorFlip(frc::Color inputcolor){
    return frc::Color(inputcolor.green, inputcolor.red, inputcolor.blue);
}