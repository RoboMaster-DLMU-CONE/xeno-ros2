#ifndef XENO_SUCK_HPP
#define XENO_SUCK_HPP
#include <memory>
#include <tl/expected.hpp>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Suck
    {
    public:
        static Suck& getInstance();
        Suck(Suck&) = delete;
        void posAngControl(float pos, float ang) const;
        [[nodiscard]] tl::expected<void, OneMotor::Error> enable();
        [[nodiscard]] tl::expected<void, OneMotor::Error> disable();
        
        // Getter methods for position/velocity reading and calibration values
        [[nodiscard]] tl::expected<float, OneMotor::Error> getPosition() const;
        [[nodiscard]] tl::expected<float, OneMotor::Error> getVelocity() const;
        [[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationPosition() const;
        [[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationVelocity() const;
        
        // Setter methods for calibration value modification
        [[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationPosition(float position);
        [[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationVelocity(float velocity);
        
        Suck& operator=(const Suck&) = delete;

    private:
        Suck();
        std::unique_ptr<OneMotor::Motor::DJI::M3508<2, OneMotor::Motor::DJI::MotorMode::Position>> m3508_;
    };
}

#endif //XENO_SUCK_HPP
