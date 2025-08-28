#include "xeno_control/joints/Shift.hpp"

using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;
using OneMotor::Control::PID_Params;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 1.5,
    .Ki = 0.005,
    .Kd = 0,
    .MaxOutput = 10000,
    .Deadband = 100,
    .IntegralLimit = 500,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 3,
    .Ki = 0.2,
    .Kd = 0.05,
    .MaxOutput = 15000,
    .Deadband = 200,
    .IntegralLimit = 2000,
};

xeno_control::Shift::Shift() = default;

void xeno_control::Shift::init(CanDriver& driver)
{
    m3508_ = std::make_unique<M3508<1, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    (void)m3508_->enable().or_else([](const auto& e)
    {
        throw std::runtime_error(e.message);
    });
    m3508_->setAngRef(50);
}

tl::expected<void, OneMotor::Error> xeno_control::Shift::enable()
{
    return m3508_->enable();
}

tl::expected<void, OneMotor::Error> xeno_control::Shift::disable()
{
    return m3508_->disable();
}

void xeno_control::Shift::writeCommand(const float command) noexcept
{
    m3508_->setPosRef(command);
}

std::pair<float, float> xeno_control::Shift::readAngPos() const noexcept
{
    auto status = m3508_->getStatus();
    return {status.angular, status.total_angle};
}
