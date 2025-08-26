#ifndef XENO_CANDRIVERMANAGER_HPP
#define XENO_CANDRIVERMANAGER_HPP

#include <memory>
#include <OneMotor/Can/CanDriver.hpp>

namespace xeno_control
{
    class CanDriverManager
    {
    public:
        static CanDriverManager& getInstance();
        [[nodiscard]] OneMotor::Can::CanDriver& getBaseDriver() const;
        [[nodiscard]] OneMotor::Can::CanDriver& getArmDriver() const;
        CanDriverManager(CanDriverManager&) = delete;
        CanDriverManager& operator=(const CanDriverManager&) = delete;
    private:
        CanDriverManager();
        std::unique_ptr<OneMotor::Can::CanDriver> base_driver;
        std::unique_ptr<OneMotor::Can::CanDriver> arm_driver;
    };
}

#endif //XENO_CANDRIVERMANAGER_HPP
