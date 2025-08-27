#ifndef STRETCH_HPP
#define STRETCH_HPP
#include <memory>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Stretch
    {
    public:
        Stretch();
        void init(OneMotor::Can::CanDriver& driver);
        Stretch(Stretch&) = delete;
        Stretch& operator=(const Stretch&) = delete;
        void posAngControl(float pos, float ang) const;

    private:
        std::unique_ptr<OneMotor::Motor::DJI::M3508<1, OneMotor::Motor::DJI::MotorMode::Position>> m3508_1;
        std::unique_ptr<OneMotor::Motor::DJI::M3508<2, OneMotor::Motor::DJI::MotorMode::Position>> m3508_2;
    };
}

#endif //STRETCH_HPP
