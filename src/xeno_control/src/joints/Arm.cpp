#include "xeno_control/joints/Arm.hpp"

#include <stdexcept>

using OneMotor::Motor::DM::J4310;
using OneMotor::Can::CanDriver;
using Result = tl::expected<void, OneMotor::Error>;

xeno_control::Arm& xeno_control::Arm::getInstance()
{
    static Arm _instance;
    return _instance;
}

Result xeno_control::Arm::posVelControl(const uint8_t id, const float position, const float velocity) const
{
    return j4310_array_[id]->posVelControl(position, velocity);
}

xeno_control::Arm::Arm()
{
    driver_ = std::make_unique<CanDriver>("can0");
    if (const auto result = driver_->open(); !result)
    {
        throw std::runtime_error(result.error().message);
    }
    for (int i = 0; i < 3; i++)
    {
        j4310_array_[i] = std::make_unique<J4310>(*driver_, 0x51 + i, 0x41 + i);
        auto result = j4310_array_[i]->setZeroPosition();
        if (!result) throw std::runtime_error(result.error().message);
    }
}

xeno_control::Arm::~Arm() = default;
