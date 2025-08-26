#include "xeno_control/joints/Lift.hpp"

#include <mutex>

#include "xeno_control/joints/CanDriverManager.hpp"

using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;
using OneMotor::Control::PID_Params;

using std::chrono_literals::operator ""ms;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 2.2,
    .Ki = 0.008,
    .Kd = 0,
    .MaxOutput = 5000,
    .Deadband = 50,
    .IntegralLimit = 150,
    .DerivativeFilterRC = 0.1,
    .OutputFilterRC = 0.02,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 6.5,
    .Ki = 0.04,
    .Kd = 0.08,
    .MaxOutput = 18000,
    .Deadband = 200,
    .IntegralLimit = 1500,
    .DerivativeFilterRC = 0.08,
    .OutputFilterRC = 0.05,
};

xeno_control::Lift& xeno_control::Lift::getInstance()
{
    static Lift _instance;
    return _instance;
}

void xeno_control::Lift::posAngControl(const float pos, const float ang) const noexcept
{
    m3508_1->setPosRef(pos);
    m3508_1->setAngRef(ang);
    m3508_2->setPosRef(-pos);
    m3508_2->setAngRef(-ang);
}

xeno_control::Lift::Lift()
{
    auto& driver = CanDriverManager::getInstance().getBaseDriver();
    m3508_1 = std::make_unique<M3508<3, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m3508_2 = std::make_unique<M3508<4, Position>>(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    (void)m3508_1->enable()
                 .and_then([this] { return m3508_2->enable(); })
                 .or_else([](const auto& e) -> tl::expected<void, OneMotor::Error>
                 {
                     throw std::runtime_error(e.message);
                 });
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::enable()
{
    return m3508_1->enable()
                  .and_then([this] { return m3508_2->enable(); });
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::disable()
{
    return m3508_1->disable()
                  .and_then([this] { return m3508_2->disable(); });
}

tl::expected<float, OneMotor::Error> xeno_control::Lift::getPosition() const
{
    return m3508_1->getPosition();
}

tl::expected<float, OneMotor::Error> xeno_control::Lift::getVelocity() const
{
    return m3508_1->getVelocity();
}

tl::expected<float, OneMotor::Error> xeno_control::Lift::getCalibrationPosition() const
{
    return m3508_1->getCalibrationPosition();
}

tl::expected<float, OneMotor::Error> xeno_control::Lift::getCalibrationVelocity() const
{
    return m3508_1->getCalibrationVelocity();
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::setCalibrationPosition(const float position)
{
    return m3508_1->setCalibrationPosition(position)
                  .and_then([this, position] { return m3508_2->setCalibrationPosition(-position); });
}

tl::expected<void, OneMotor::Error> xeno_control::Lift::setCalibrationVelocity(const float velocity)
{
    return m3508_1->setCalibrationVelocity(velocity)
                  .and_then([this, velocity] { return m3508_2->setCalibrationVelocity(-velocity); });
}
