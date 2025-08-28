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
    m3508_1->setAngRef(80);
    m3508_2->setAngRef(80);
}

tl::expected<void, OneMotor::Error> xeno_control::Stretch::enable()
{
    return m3508_1->enable()
                   .and_then([this] { return m3508_2->enable(); });
}

tl::expected<void, OneMotor::Error> xeno_control::Stretch::disable()
{
    return m3508_1->disable()
                   .and_then([this] { return m3508_2->disable(); });
}

void xeno_control::Stretch::writeCommand(const float command) noexcept
{
    m3508_1->setPosRef(command);
    m3508_2->setPosRef(-command);
}

std::pair<float, float> xeno_control::Stretch::readAngPos() const noexcept
{
    auto status = m3508_1->getStatus();
    return {status.angular, status.total_angle};
}
