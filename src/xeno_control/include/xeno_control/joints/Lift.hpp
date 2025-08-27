#ifndef XENO_CONTROL_LIFT_HPP
#define XENO_CONTROL_LIFT_HPP

#include <memory>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Lift
    {
    public:
        Lift();
        void init(OneMotor::Can::CanDriver& driver);
        tl::expected<void, OneMotor::Error> disable();
        tl::expected<void, OneMotor::Error> enable();
        void posAngControl(float pos, float ang) const noexcept;
        Lift(Lift&) = delete;
        Lift& operator=(const Lift&) = delete;
        ~Lift();

    private:
        std::unique_ptr<OneMotor::Motor::DJI::M3508<3, OneMotor::Motor::DJI::MotorMode::Position>> m3508_1;
        std::unique_ptr<OneMotor::Motor::DJI::M3508<4, OneMotor::Motor::DJI::MotorMode::Position>> m3508_2;
    };
}

#endif //XENO_CONTROL_LIFT_HPP
