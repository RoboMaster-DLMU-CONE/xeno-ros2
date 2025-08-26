#ifndef XENO_LIFTSTRETCH_HPP
#define XENO_LIFTSTRETCH_HPP
#include <memory>
#include <tl/expected.hpp>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Lift
    {
    public:
        static Lift& getInstance();
        void posAngControl(float pos, float ang) const noexcept;
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
        
        Lift(Lift&) = delete;
        Lift& operator=(const Lift&) = delete;

    private:
        Lift();
        std::unique_ptr<OneMotor::Motor::DJI::M3508<3, OneMotor::Motor::DJI::MotorMode::Position>> m3508_1;
        std::unique_ptr<OneMotor::Motor::DJI::M3508<4, OneMotor::Motor::DJI::MotorMode::Position>> m3508_2;
    };
}

#endif //XENO_LIFTSTRETCH_HPP
