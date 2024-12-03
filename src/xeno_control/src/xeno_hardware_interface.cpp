#include "xeno_control/xeno_hardware_interface.hpp"
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace xeno_control {
  CallbackReturn XenoHardware::on_init(const hardware_interface::HardwareInfo &info) {
    // 初始化硬件接口
    // 例如，连接到电机控制器
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> XenoHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // 添加电机位置和速度状态接口
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_1", hardware_interface::HW_IF_POSITION, &joints[1].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_1", hardware_interface::HW_IF_VELOCITY, &joints[1].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_2", hardware_interface::HW_IF_POSITION, &joints[2].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_2", hardware_interface::HW_IF_VELOCITY, &joints[2].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_3", hardware_interface::HW_IF_POSITION, &joints[3].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_3", hardware_interface::HW_IF_VELOCITY, &joints[3].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_4", hardware_interface::HW_IF_POSITION, &joints[4].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_4", hardware_interface::HW_IF_VELOCITY, &joints[4].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_5", hardware_interface::HW_IF_POSITION, &joints[5].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_5", hardware_interface::HW_IF_VELOCITY, &joints[5].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_6", hardware_interface::HW_IF_POSITION, &joints[6].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface("joint_6", hardware_interface::HW_IF_VELOCITY, &joints[6].velocity));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> XenoHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // 添加电机位置命令接口
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_1", hardware_interface::HW_IF_POSITION, &joints[1].command));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_2", hardware_interface::HW_IF_POSITION, &joints[2].command));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_3", hardware_interface::HW_IF_POSITION, &joints[3].command));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_4", hardware_interface::HW_IF_POSITION, &joints[4].command));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_5", hardware_interface::HW_IF_POSITION, &joints[5].command));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface("joint_6", hardware_interface::HW_IF_POSITION, &joints[6].command));
    return command_interfaces;
  }

  CallbackReturn XenoHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    // 激活硬件接口
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn XenoHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    // 停用硬件接口
    return CallbackReturn::SUCCESS;
  }

  return_type XenoHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // 从硬件读取电机位置和速度
    for (int i = 1; i <= 6; i++) {
      joints[i].position = read_motor_position(i);
      joints[i].velocity = read_motor_velocity(i);
      //   RCLCPP_INFO(
      //     rclcpp::get_logger("XenoHardware"), std::to_string(joints[i].position).c_str());
    }

    return hardware_interface::return_type::OK;
  }

  return_type XenoHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // 将命令写入硬件
    for (int i = 1; i <= 6; i++) {
      write_motor_position(i, joints[i].command);
    }
    return hardware_interface::return_type::OK;
  }
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(xeno_control::XenoHardware, hardware_interface::SystemInterface);



