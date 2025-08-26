#ifndef XENO_STRETCH_HPP
#define XENO_STRETCH_HPP
#include <memory>
#include <tl/expected.hpp>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Stretch
    {
    public:
        static Stretch& getInstance();
        Stretch(Stretch&) = delete;
        Stretch& operator=(const Stretch&) = delete;
        void posAngControl(float pos, float ang) const;
        [[nodiscard]] tl::expected<void, OneMotor::Error> enable();
        [[nodiscard]] tl::expected<void, OneMotor::Error> disable();

    private:
        Stretch();
        std::unique_ptr<OneMotor::Motor::DJI::M3508<1, OneMotor::Motor::DJI::MotorMode::Position>> m3508_1;
        std::unique_ptr<OneMotor::Motor::DJI::M3508<2, OneMotor::Motor::DJI::MotorMode::Position>> m3508_2;
    };
}

#endif //XENO_STRETCH_HPP
