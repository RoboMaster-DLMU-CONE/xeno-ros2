#ifndef SUCK_HPP
#define SUCK_HPP
#include <memory>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Suck
    {
    public:
        Suck();
        void init(OneMotor::Can::CanDriver& driver);
        Suck(Suck&) = delete;
        void posAngControl(float pos, float ang) const;
        Suck& operator=(const Suck&) = delete;

    private:
        std::unique_ptr<OneMotor::Motor::DJI::M3508<2, OneMotor::Motor::DJI::MotorMode::Position>> m3508_;
    };
}

#endif //SUCK_HPP
