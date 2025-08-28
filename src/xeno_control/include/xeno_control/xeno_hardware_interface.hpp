#ifndef XENO_HARDWARE_HPP_
#define XENO_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "joints/Arm.hpp"
#include "joints/Lift.hpp"
#include "joints/Stretch.hpp"
#include "joints/Shift.hpp"
#include "joints/Suck.hpp"


using hardware_interface::return_type;

namespace xeno_control
{
  struct Joint
  {
    Joint()
    {
      position = 0.0;
      velocity = 0.0;
      command = 0.0;
    }

    double position;
    double velocity;
    double command;
  };

  class XenoHardware final : public hardware_interface::SystemInterface
  {
  public:
    XenoHardware();
    ~XenoHardware() override;
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

  private:
    Joint joints[8];

#ifdef XENO_CONTROL_SIMULATE

    double read_motor_position(int joint_id) const;

    double read_motor_velocity(int joint_id) const;

    void write_motor_position(int joint_id, double position);

#else
    OneMotor::Can::CanDriver* base_driver;
    OneMotor::Can::CanDriver* arm_driver;

    Lift* lift;
    Arm* arm;
    Stretch* stretch;
    Shift* shift;
    Suck* suck;
#endif
  };
}

#endif
