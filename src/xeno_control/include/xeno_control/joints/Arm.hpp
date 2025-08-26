#ifndef XENO_ARM_HPP
#define XENO_ARM_HPP
#include <tl/expected.hpp>
#include <memory>
#include <OneMotor/Motor/DM/J4310.hpp>

namespace xeno_control
{
    class Arm
    {
    public:
        static Arm& getInstance();
        [[nodiscard]] tl::expected<void, OneMotor::Error> posVelControl(uint8_t id, float position,
                                                                        float velocity) const;
        [[nodiscard]] tl::expected<void, OneMotor::Error> enable();
        [[nodiscard]] tl::expected<void, OneMotor::Error> disable();
        Arm(Arm&) = delete;
        Arm& operator=(const Arm&) = delete;
        ~Arm();

    private:
        Arm();
        std::unique_ptr<OneMotor::Can::CanDriver> driver_;
        std::array<std::unique_ptr<OneMotor::Motor::DM::J4310>, 3> j4310_array_;
    };
}

#endif //XENO_ARM_HPP
