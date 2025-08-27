#ifndef SHIFT_HPP
#define SHIFT_HPP
#include <memory>
#include <OneMotor/Motor/DJI/M3508.hpp>

namespace xeno_control
{
    class Shift
    {
    public:
        Shift();
        void init(OneMotor::Can::CanDriver& driver);
        void posAngControl(float pos, float ang) const;
        Shift(Shift&) = delete;
        Shift& operator=(const Shift&) = delete;
        ~Shift() = default;

    private:
        std::unique_ptr<OneMotor::Motor::DJI::M3508<1, OneMotor::Motor::DJI::MotorMode::Position>> m3508_;
    };
}

#endif //SHIFT_HPP
