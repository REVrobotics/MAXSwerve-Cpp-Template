#include <frc/LEDPattern.h>
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

        // Our LED strip has a density of 60 LEDs per meter
        units::meter_t kLedSpacing{1 / 60.0};
        std::array<Color, 2> colors{ColorFlip(Color::kRed), ColorFlip(Color::kBlue)};
        LEDPattern gradient = LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
        LEDPattern redlight = LEDPattern::Solid(ColorFlip(Color::kRed));
        LEDPattern greenlight = LEDPattern::Solid(ColorFlip(Color::kGreen));
        LEDPattern purplelight = LEDPattern::Solid(ColorFlip(Color::kPurple));
        LEDPattern bluelight = LEDPattern::Solid(ColorFlip(Color::kBlue));
        // Create an LED pattern that will display a rainbow across
        // all hues at maximum saturation and half brightness
        LEDPattern m_rainbow = LEDPattern::Rainbow(255, 128);
        
        // Create a new pattern that scrolls the rainbow pattern across the LED
        // strip, moving at a speed of 1 meter per second.
        frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(.5_mps, kLedSpacing);
        //m_scrollingRainbow = m_rainbow.Breathe(3_s);
        //m_scrollingRainbow = m_rainbow.Mask();
        
        // Create an LED pattern that displays a red-to-blue gradient.
        // The LED strip will be red at both ends and blue in the center,
        // with smooth gradients between
        std::array<Color, 2> colors{ColorFlip(Color::kRed), ColorFlip(Color::kBlue)};
        LEDPattern gradient = LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
  
        std::array<std::pair<double, Color>, 2> colorSteps{std::pair{0.0, ColorFlip(Color::kRed)},
                                                            std::pair{0.5, Color::kBlue}};
        LEDPattern steps = LEDPattern::Steps(colorSteps);

        //std::array<Color, 2> colors{ColorFlip(Color::kRed), Color::kBlue};
        //LEDPattern base = LEDPattern::LEDPattern::Gradient(LEDPattern::GradientType::kDiscontinuous, colors);
        LEDPattern base = LEDPattern::Steps(colorSteps);
        LEDPattern pattern = base.ScrollAtRelativeSpeed(units::hertz_t{0.25});
        LEDPattern absolute = base.ScrollAtAbsoluteSpeed(0.125_mps, units::meter_t{1/120.0});

        


    private:
        static constexpr int kPort = 9;
        static constexpr int kLength = 300;
        frc::AddressableLED m_led{kPort};
        std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
};