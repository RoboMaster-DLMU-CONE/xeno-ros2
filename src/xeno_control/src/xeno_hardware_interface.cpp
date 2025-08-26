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
        for (int i = 1; i <= 7; i++)
        {
            joints[i].position = read_motor_position(i);
            joints[i].velocity = read_motor_velocity(i);
            //   RCLCPP_INFO(
            //     rclcpp::get_logger("XenoHardware"), std::to_string(joints[i].position).c_str());
        }
#else
        // Joint 1: Lift
        auto lift_pos = Lift::getInstance().getPosition();
        auto lift_vel = Lift::getInstance().getVelocity();
        if (lift_pos.has_value()) joints[1].position = lift_pos.value();
        if (lift_vel.has_value()) joints[1].velocity = lift_vel.value();
        
        // Joint 2: Stretch  
        auto stretch_pos = Stretch::getInstance().getPosition();
        auto stretch_vel = Stretch::getInstance().getVelocity();
        if (stretch_pos.has_value()) joints[2].position = stretch_pos.value();
        if (stretch_vel.has_value()) joints[2].velocity = stretch_vel.value();
        
        // Joint 3: Shift
        auto shift_pos = Shift::getInstance().getPosition();
        auto shift_vel = Shift::getInstance().getVelocity();
        if (shift_pos.has_value()) joints[3].position = shift_pos.value();
        if (shift_vel.has_value()) joints[3].velocity = shift_vel.value();
        
        // Joints 4-6: Arm[1-3]
        for (int i = 1; i <= 3; i++)
        {
            auto arm_pos = Arm::getInstance().getPosition(i);
            auto arm_vel = Arm::getInstance().getVelocity(i);
            if (arm_pos.has_value()) joints[3 + i].position = arm_pos.value();
            if (arm_vel.has_value()) joints[3 + i].velocity = arm_vel.value();
        }
        
        // Joint 7: Suck
        auto suck_pos = Suck::getInstance().getPosition();
        auto suck_vel = Suck::getInstance().getVelocity();
        if (suck_pos.has_value()) joints[7].position = suck_pos.value();
        if (suck_vel.has_value()) joints[7].velocity = suck_vel.value();
#endif

        return return_type::OK;
    }

    return_type XenoHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 将命令写入硬件
#ifdef XENO_CONTROL_SIMULATE
        (void)time;
        (void)period;
        for (int i = 1; i <= 7; i++)
        {
            write_motor_position(i, joints[i].command);
        }
        return return_type::OK;
#else
        (void)time;
        (void)period;
        
        // Joint 1: Lift - use position and angular control
        Lift::getInstance().posAngControl(joints[1].command, 0.0);
        
        // Joint 2: Stretch - use position and angular control
        Stretch::getInstance().posAngControl(joints[2].command, 0.0);
        
        // Joint 3: Shift - use position and angular control
        Shift::getInstance().posAngControl(joints[3].command, 0.0);
        
        // Joints 4-6: Arm[1-3] - use position and velocity control
        for (int i = 1; i <= 3; i++)
        {
            auto result = Arm::getInstance().posVelControl(i, joints[3 + i].command, 0.0);
            if (!result.has_value())
            {
                RCLCPP_ERROR(rclcpp::get_logger("XenoHardware"), 
                           "Failed to control Arm joint %d: %s", i, result.error().message.c_str());
            }
        }
        
        // Joint 7: Suck - use position and angular control
        Suck::getInstance().posAngControl(joints[7].command, 0.0);
        
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

    bool XenoHardware::getCalibrationPosition(const int joint_id, double& position) const
    {
        switch (joint_id)
        {
            case 1: // Lift
            {
                auto result = Lift::getInstance().getCalibrationPosition();
                if (result.has_value()) {
                    position = result.value();
                    return true;
                }
                break;
            }
            case 2: // Stretch
            {
                auto result = Stretch::getInstance().getCalibrationPosition();
                if (result.has_value()) {
                    position = result.value();
                    return true;
                }
                break;
            }
            case 3: // Shift
            {
                auto result = Shift::getInstance().getCalibrationPosition();
                if (result.has_value()) {
                    position = result.value();
                    return true;
                }
                break;
            }
            case 4: case 5: case 6: // Arm[1-3]
            {
                auto result = Arm::getInstance().getCalibrationPosition(joint_id - 3);
                if (result.has_value()) {
                    position = result.value();
                    return true;
                }
                break;
            }
            case 7: // Suck
            {
                auto result = Suck::getInstance().getCalibrationPosition();
                if (result.has_value()) {
                    position = result.value();
                    return true;
                }
                break;
            }
        }
        return false;
    }

    bool XenoHardware::getCalibrationVelocity(const int joint_id, double& velocity) const
    {
        switch (joint_id)
        {
            case 1: // Lift
            {
                auto result = Lift::getInstance().getCalibrationVelocity();
                if (result.has_value()) {
                    velocity = result.value();
                    return true;
                }
                break;
            }
            case 2: // Stretch
            {
                auto result = Stretch::getInstance().getCalibrationVelocity();
                if (result.has_value()) {
                    velocity = result.value();
                    return true;
                }
                break;
            }
            case 3: // Shift
            {
                auto result = Shift::getInstance().getCalibrationVelocity();
                if (result.has_value()) {
                    velocity = result.value();
                    return true;
                }
                break;
            }
            case 4: case 5: case 6: // Arm[1-3]
            {
                auto result = Arm::getInstance().getCalibrationVelocity(joint_id - 3);
                if (result.has_value()) {
                    velocity = result.value();
                    return true;
                }
                break;
            }
            case 7: // Suck
            {
                auto result = Suck::getInstance().getCalibrationVelocity();
                if (result.has_value()) {
                    velocity = result.value();
                    return true;
                }
                break;
            }
        }
        return false;
    }

    bool XenoHardware::setCalibrationPosition(const int joint_id, const double position)
    {
        switch (joint_id)
        {
            case 1: // Lift
                return Lift::getInstance().setCalibrationPosition(position).has_value();
            case 2: // Stretch
                return Stretch::getInstance().setCalibrationPosition(position).has_value();
            case 3: // Shift
                return Shift::getInstance().setCalibrationPosition(position).has_value();
            case 4: case 5: case 6: // Arm[1-3]
                return Arm::getInstance().setCalibrationPosition(joint_id - 3, position).has_value();
            case 7: // Suck
                return Suck::getInstance().setCalibrationPosition(position).has_value();
        }
        return false;
    }

    bool XenoHardware::setCalibrationVelocity(const int joint_id, const double velocity)
    {
        switch (joint_id)
        {
            case 1: // Lift
                return Lift::getInstance().setCalibrationVelocity(velocity).has_value();
            case 2: // Stretch
                return Stretch::getInstance().setCalibrationVelocity(velocity).has_value();
            case 3: // Shift
                return Shift::getInstance().setCalibrationVelocity(velocity).has_value();
            case 4: case 5: case 6: // Arm[1-3]
                return Arm::getInstance().setCalibrationVelocity(joint_id - 3, velocity).has_value();
            case 7: // Suck
                return Suck::getInstance().setCalibrationVelocity(velocity).has_value();
        }
        return false;
    }
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(xeno_control::XenoHardware, hardware_interface::SystemInterface);
