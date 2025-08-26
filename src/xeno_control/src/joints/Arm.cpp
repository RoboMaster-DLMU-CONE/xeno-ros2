#include "xeno_control/joints/Arm.hpp"

using OneMotor::Motor::DM::J4310;
using OneMotor::Can::CanDriver;

xeno_control::Arm& xeno_control::Arm::getInstance()
{
    static Arm _instance;
    return _instance;
}

tl::expected<void, OneMotor::Error> xeno_control::Arm::posVelControl(const uint8_t id, const float position,
                                                                     const float velocity) const
{
    return j4310_array_[id - 1]->posVelControl(position, velocity);
}

xeno_control::Arm::Arm()
{
    driver_ = std::make_unique<CanDriver>("can0");
    (void)driver_->open().or_else([](const auto& e)
    {
        throw std::runtime_error(e.message);
    });
    for (int i = 0; i < 3; i++)
    {
        j4310_array_[i] = std::make_unique<J4310>(*driver_, 0x51 + i, 0x41 + i);
        (void)j4310_array_[i]->enable().and_then([&] { return j4310_array_[i]->setZeroPosition(); });
    }
}

xeno_control::Arm::~Arm() = default;

tl::expected<void, OneMotor::Error> xeno_control::Arm::enable()
{
    auto result = j4310_array_[0]->enable();
    for (int i = 1; i < 3; i++)
    {
        result = result.and_then([this, i] { return j4310_array_[i]->enable(); });
        if (!result) break;
    }
    return result;
}

tl::expected<void, OneMotor::Error> xeno_control::Arm::disable()
{
    auto result = j4310_array_[0]->disable();
    for (int i = 1; i < 3; i++)
    {
        result = result.and_then([this, i] { return j4310_array_[i]->disable(); });
        if (!result) break;
    }
    return result;
}
