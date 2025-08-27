#include "xeno_control/joints/Stretch.hpp"

using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;
using OneMotor::Control::PID_Params;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 1.8,
    .Ki = 0.005,
    .Kd = 0,
    .MaxOutput = 20000,
    .Deadband = 150,
    .IntegralLimit = 150,
    .DerivativeFilterRC = 0.1,
    .OutputFilterRC = 0.02,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 4.5,
    .Ki = 0.5,
    .Kd = 0.1,
    .MaxOutput = 20000,
    .Deadband = 250,
    .IntegralLimit = 4000,
};

void xeno_control::Stretch::posAngControl(const float pos, const float ang) const
{
    m3508_1->setPosRef(pos);
    m3508_1->setAngRef(ang);
    m3508_2->setPosRef(-pos);
    m3508_2->setAngRef(-ang);
}

xeno_control::Stretch::Stretch() = default;

void xeno_control::Stretch::init(CanDriver& driver)
{
    m3508_1 = std::make_unique<M3508<1, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m3508_2 = std::make_unique<M3508<2, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    (void)m3508_1->enable()
                  .and_then([this] { return m3508_2->enable(); })
                  .or_else([](const auto& e) -> tl::expected<void, OneMotor::Error>
                   {
                       throw std::runtime_error(e.message);
                   });
}
