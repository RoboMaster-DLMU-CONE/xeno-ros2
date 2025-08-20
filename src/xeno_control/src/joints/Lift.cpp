#include "xeno_control/joints/Lift.hpp"

#include <stdexcept>

using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;
using OneMotor::Control::PID_Params;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 6,
    .Ki = 0,
    .Kd = 0,
    .MaxOutput = 2000,
    .Deadband = 30,
    .IntegralLimit = 500,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 5,
    .Ki = 0.3,
    .Kd = 0,
    .MaxOutput = 15000,
    .Deadband = 30,
    .IntegralLimit = 1500,
};

xeno_control::Lift& xeno_control::Lift::getInstance()
{
    static Lift _instance;
    return _instance;
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::disable()
{
    return m3508_1->disable().and_then([&]
    {
        return m3508_2->disable();
    });
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::enable()
{
    return m3508_1->enable().and_then([&]
    {
        return m3508_2->enable();
    });
}

void xeno_control::Lift::posAngControl(const float pos, const float ang) const noexcept
{
    m3508_1->setPosRef(-pos);
    m3508_1->setAngRef(-ang);
    m3508_2->setPosRef(pos);
    m3508_2->setAngRef(ang);
}

xeno_control::Lift::Lift()
{
    driver_ = std::make_unique<CanDriver>("can1");
    m3508_1 = std::make_unique<M3508<3, Position>>(*driver_, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m3508_2 = std::make_unique<M3508<4, Position>>(*driver_, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
}

xeno_control::Lift::~Lift() = default;
