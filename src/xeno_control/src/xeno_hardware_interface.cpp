#include "xeno_control/xeno_hardware_interface.hpp"
#include "xeno_control/joints/Arm.hpp"
#include "xeno_control/joints/Lift.hpp"
#include "xeno_control/joints/Stretch.hpp"
#include "xeno_control/joints/Shift.hpp"
#include "xeno_control/joints/Suck.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace xeno_control
{
    CallbackReturn XenoHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        // 初始化硬件接口
        // 例如，连接到电机控制器
#ifdef XENO_CONTROL_SIMULATE
#else
        Lift::getInstance();
        Stretch::getInstance();
        Shift::getInstance();
        Suck::getInstance();
        Arm::getInstance();

#endif
        (void)info;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn XenoHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        // 激活硬件接口
        (void)previous_state;
#ifdef XENO_CONTROL_SIMULATE
        return CallbackReturn::SUCCESS;
#else
        return Lift::getInstance().enable()
                                  .and_then([] { return Stretch::getInstance().enable(); })
                                  .and_then([] { return Shift::getInstance().enable(); })
                                  .and_then([] { return Suck::getInstance().enable(); })
                                  .and_then([] { return Arm::getInstance().enable(); })
                                  .map([] { return CallbackReturn::SUCCESS; })
                                  .value_or(CallbackReturn::ERROR);

#endif
    }

    CallbackReturn XenoHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        (void)previous_state;
#ifdef XENO_CONTROL_SIMULATE
        // 停用硬件接口
        return CallbackReturn::SUCCESS;
#else
        return Lift::getInstance().disable()
                                  .and_then([] { return Stretch::getInstance().disable(); })
                                  .and_then([] { return Shift::getInstance().disable(); })
                                  .and_then([] { return Suck::getInstance().disable(); })
                                  .and_then([] { return Arm::getInstance().disable(); })
                                  .map([] { return CallbackReturn::SUCCESS; })
                                  .value_or(CallbackReturn::ERROR);
#endif
    }

    return_type XenoHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 从硬件读取电机位置和速度
        (void)time;
        (void)period;
#ifdef XENO_CONTROL_SIMULATE
        for (int i = 1; i <= 6; i++)
        {
            joints[i].position = read_motor_position(i);
            joints[i].velocity = read_motor_velocity(i);
            //   RCLCPP_INFO(
            //     rclcpp::get_logger("XenoHardware"), std::to_string(joints[i].position).c_str());
        }
#endif


        return return_type::OK;
    }

    return_type XenoHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 将命令写入硬件
#ifdef XENO_CONTROL_SIMULATE
        (void)time;
        (void)period;
        for (int i = 1; i <= 6; i++)
        {
            write_motor_position(i, joints[i].command);
        }
        return return_type::OK;
#endif
    }

    double XenoHardware::read_motor_position(const int joint_id) const
    {
        return joints[joint_id].position;
    }

    double XenoHardware::read_motor_velocity(const int joint_id) const
    {
        return joints[joint_id].velocity;
    }

    void XenoHardware::write_motor_position(const int joint_id, const double position)
    {
        joints[joint_id].position = position;
    }
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(xeno_control::XenoHardware, hardware_interface::SystemInterface);
