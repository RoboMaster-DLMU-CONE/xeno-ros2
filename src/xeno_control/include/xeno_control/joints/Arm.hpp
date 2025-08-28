#ifndef XENO_CONTROL_ARM_HPP
#define XENO_CONTROL_ARM_HPP
#include <memory>
#include <OneMotor/Motor/DM/J4310.hpp>

namespace xeno_control
{
    class Arm
    {
    public:
        Arm();
        void init(OneMotor::Can::CanDriver& driver);

        [[nodiscard]] tl::expected<void, OneMotor::Error> posVelControl(uint8_t id, float position,
                                                                        float velocity) const;
        tl::expected<void, OneMotor::Error> enable();
        tl::expected<void, OneMotor::Error> disable();

        void writeCommand(float command, uint8_t id) noexcept;

        Arm(Arm&) = delete;
        Arm& operator=(const Arm&) = delete;
        ~Arm();

        std::array<float, 3> pos{};
        std::array<float, 3> ang{};

    private:
        std::array<std::unique_ptr<OneMotor::Motor::DM::J4310>, 3> j4310_array_;
    };
}
#endif //XENO_CONTROL_ARM_HPP
