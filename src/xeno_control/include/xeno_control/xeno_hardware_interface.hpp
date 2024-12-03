#ifndef XENO_HARDWARE_HPP_
#define XENO_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>


using hardware_interface::return_type;

namespace xeno_control {
  struct Joint {
    Joint() {
      position = 0.0;
      velocity = 0.0;
      command = 0.0;
    }

    double position;
    double velocity;
    double command;
  };

  class XenoHardware : public hardware_interface::SystemInterface {
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    Joint joints[7];

    double read_motor_position(const int joint_id) const {
      return joints[joint_id].position;
    }

    double read_motor_velocity(const int joint_id) const {
      return joints[joint_id].velocity;
    }

    void write_motor_position(const int joint_id, const double position) {
      joints[joint_id].position = position;
    }
  };
}

#endif
