#include "xeno_control/xeno_hardware_interface.hpp"
#include "xeno_control/joints/Arm.hpp"
#include "xeno_control/joints/Lift.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace xeno_control
{
  XenoHardware::XenoHardware()
  {
#ifdef XENO_CONTROL_SIMULATE
#else
    base_driver = new OneMotor::Can::CanDriver("can0");
    arm_driver = new OneMotor::Can::CanDriver("can1");
    lift = new Lift();
    arm = new Arm();
    stretch = new Stretch();
    shift = new Shift();
    suck = new Suck();
#endif
  }

  XenoHardware::~XenoHardware()
  {
#ifdef XENO_CONTROL_SIMULATE
#else
    delete base_driver;
    delete arm_driver;

    delete lift;
    delete arm;
    delete stretch;
    delete shift;
    delete suck;
#endif
  }

  CallbackReturn XenoHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    // 初始化硬件接口
    // 例如，连接到电机控制器
#ifdef XENO_CONTROL_SIMULATE
#else
    lift->init(*base_driver);
    arm->init(*arm_driver);
    stretch->init(*base_driver);
    shift->init(*base_driver);
    suck->init(*base_driver);
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
    return lift->enable()
                .and_then([this] { return stretch->enable(); })
                .and_then([this] { return shift->enable(); })
                .and_then([this] { return suck->enable(); })
                .and_then([this] { return arm->enable(); })
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
    return lift->disable()
                .and_then([this] { return stretch->disable(); })
                .and_then([this] { return shift->disable(); })
                .and_then([this] { return suck->disable(); })
                .and_then([this] { return arm->disable(); })
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
      RCLCPP_INFO(
        rclcpp::get_logger("XenoHardware"), std::to_string(joints[i].position).c_str());
    }
#else
    auto data = lift->readAngPos();
    joints[1].velocity = data.first;
    joints[1].position = data.second;

    data = stretch->readAngPos();
    joints[2].velocity = data.first;
    joints[2].position = data.second;

    data = shift->readAngPos();
    joints[3].velocity = data.first;
    joints[3].position = data.second;

    joints[4].position = arm->pos[0];
    joints[5].position = arm->pos[1];
    joints[6].position = arm->pos[2];
    joints[4].velocity = arm->ang[0];
    joints[5].velocity = arm->ang[1];
    joints[6].velocity = arm->ang[2];

    data = suck->readAngPos();
    joints[7].velocity = data.first;
    joints[7].position = data.second;

#endif


    return return_type::OK;
  }

  return_type XenoHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    // 将命令写入硬件
    (void)time;
    (void)period;
#ifdef XENO_CONTROL_SIMULATE

    for (int i = 1; i <= 7; i++)
    {
      write_motor_position(i, joints[i].command);
    }
#else
    lift->writeCommand(joints[1].command);
    stretch->writeCommand(joints[2].command);
    shift->writeCommand(joints[3].command);
    arm->writeCommand(joints[4].command, 0);
    arm->writeCommand(joints[5].command, 1);
    arm->writeCommand(joints[6].command, 2);
    suck->writeCommand(joints[7].command);
#endif
    return return_type::OK;
  }
#ifdef XENO_CONTROL_SIMULATE
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
#endif
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(xeno_control::XenoHardware, hardware_interface::SystemInterface);
