#include "xeno_control/joints/CanDriverManager.hpp"

xeno_control::CanDriverManager& xeno_control::CanDriverManager::getInstance()
{
    static CanDriverManager _instance;
    return _instance;
}

OneMotor::Can::CanDriver& xeno_control::CanDriverManager::getBaseDriver() const
{
    return *base_driver;
}

OneMotor::Can::CanDriver& xeno_control::CanDriverManager::getArmDriver() const
{
    return *arm_driver;
}

xeno_control::CanDriverManager::CanDriverManager()
{
    base_driver = std::make_unique<OneMotor::Can::CanDriver>("can0");
    arm_driver = std::make_unique<OneMotor::Can::CanDriver>("can1");
    (void)base_driver->open().or_else([](const auto& e)
    {
        throw std::runtime_error(e.message);
    });
    (void)arm_driver->open().or_else([](const auto& e)
    {
        throw std::runtime_error(e.message);
    });
}
