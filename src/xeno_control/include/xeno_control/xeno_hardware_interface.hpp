#ifndef XENO_HARDWARE_HPP_
#define XENO_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>


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
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

    // Methods for accessing calibration values
    bool getCalibrationPosition(int joint_id, double& position) const;
    bool getCalibrationVelocity(int joint_id, double& velocity) const;
    bool setCalibrationPosition(int joint_id, double position);
    bool setCalibrationVelocity(int joint_id, double velocity);

  private:
    Joint joints[8];

    double read_motor_position(int joint_id) const;

    double read_motor_velocity(int joint_id) const;

    void write_motor_position(int joint_id, double position);
  };
}

#endif
