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
        tl::expected<void, OneMotor::Error> enable();
        tl::expected<void, OneMotor::Error> disable();
        void writeCommand(float command) noexcept;
        std::pair<float, float> readAngPos() const noexcept;
        Shift(Shift&) = delete;
        Shift& operator=(const Shift&) = delete;
        ~Shift() = default;

    private:
        std::unique_ptr<OneMotor::Motor::DJI::M3508<1, OneMotor::Motor::DJI::MotorMode::Position>> m3508_;
    };
}

#endif //SHIFT_HPP
