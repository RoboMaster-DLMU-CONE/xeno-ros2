#include "xeno_control/joints/Suck.hpp"

using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;
using OneMotor::Control::PID_Params;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 1.5,
    .Ki = 0.002,
    .Kd = 0,
    .MaxOutput = 3000,
    .Deadband = 100,
    .IntegralLimit = 150,
    .DerivativeFilterRC = 0.1,
    .OutputFilterRC = 0.02,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 2.0,
    .Ki = 0.2,
    .Kd = 0.05,
    .MaxOutput = 15000,
    .Deadband = 100,
    .IntegralLimit = 2000,
};

void xeno_control::Suck::posAngControl(const float pos, const float ang) const
{
    m3508_->setPosRef(pos);
    m3508_->setAngRef(ang);
}

xeno_control::Suck::Suck() = default;

void xeno_control::Suck::init(CanDriver& driver)
{
    m3508_ = std::make_unique<M3508<2, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    (void)m3508_->enable().or_else([](const auto& e)
    {
        throw std::runtime_error(e.message);
    });
}
