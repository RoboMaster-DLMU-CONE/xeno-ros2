#include "xeno_control/joints/Arm.hpp"

#include <stdexcept>

using OneMotor::Motor::DM::J4310;
using OneMotor::Can::CanDriver;
using Result = tl::expected<void, OneMotor::Error>;

Result xeno_control::Arm::posVelControl(const uint8_t id, const float position, const float velocity) const
{
    return j4310_array_[id]->posVelControl(position, velocity);
}

tl::expected<void, OneMotor::Error> xeno_control::Arm::enable()
{
    for (const auto& motor : j4310_array_)
    {
        auto result = motor->enable();
        if (!result) return result;
    }
    return {};
}

tl::expected<void, OneMotor::Error> xeno_control::Arm::disable()
{
    for (const auto& motor : j4310_array_)
    {
        auto result = motor->disable();
        if (!result) return result;
    }
    return {};
}

void xeno_control::Arm::writeCommand(const float command, const uint8_t id) noexcept
{
    (void)j4310_array_[id]->posVelControl(command, ang[id]).or_else([](const auto& err)
    {
        throw std::runtime_error(err.message);
    });
    pos[id] = command;
}

xeno_control::Arm::Arm() = default;

void xeno_control::Arm::init(CanDriver& driver)
{
    for (int i = 0; i < 3; i++)
    {
        j4310_array_[i] = std::make_unique<J4310>(driver, 0x51 + i, 0x41 + i);
        auto result = j4310_array_[i]->setZeroPosition();
        if (!result) throw std::runtime_error(result.error().message);
    }
    std::ranges::fill(ang, 20);
    std::ranges::fill(pos, 0);
}

xeno_control::Arm::~Arm() = default;
