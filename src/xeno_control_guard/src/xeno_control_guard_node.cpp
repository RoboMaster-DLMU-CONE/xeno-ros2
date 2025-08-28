#include <memory>
#include <chrono>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <OneMotor/Motor/DJI/MotorGuard.hpp>

using namespace std::chrono_literals;

class XenoControlGuardNode : public rclcpp::Node
{
public:
    XenoControlGuardNode() : Node("xeno_control_guard_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Xeno Control Guard Node");
        
#ifdef XENO_CONTROL_SIMULATE
        RCLCPP_WARN(this->get_logger(), "Running in SIMULATION mode - MotorGuard is disabled");
#else
        std::array<uint8_t, 16> exit_frame_data{};
        std::ranges::fill(exit_frame_data, 0x00);
        
        // Setup MotorGuard for both CAN interface
        std::vector<OneMotor::Motor::DJI::MotorGuard::DriverPair> driver_set;
        
        // Add can0 interface with exit frame data
        driver_set.emplace_back("can0", exit_frame_data);
        
        // Add can1 interface with exit frame data  
        driver_set.emplace_back("can1", exit_frame_data);
        
        // Start the motor guard protection
        try 
        {
            OneMotor::Motor::DJI::MotorGuard::getInstance().guard(driver_set);
            RCLCPP_INFO(this->get_logger(), "MotorGuard successfully started for can0 and can1 interfaces");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start MotorGuard: %s", e.what());
            throw;
        }
#endif
        
        // Create a timer to periodically log status
        status_timer_ = this->create_wall_timer(
            10s, [this] { status_callback(); });
    }

private:
    void status_callback()
    {
#ifdef XENO_CONTROL_SIMULATE
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000, 
                           "Motor guard running in simulation mode - hardware protection disabled");
#else
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000, 
                           "Motor guard active - protecting can0 and can1 interfaces");
#endif
    }
    
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    
    auto logger = rclcpp::get_logger("xeno_control_guard");
    
    try 
    {
        auto node = std::make_shared<XenoControlGuardNode>();
        RCLCPP_INFO(logger, "Xeno Control Guard Node started successfully");
        
        // Spin the node to keep it alive
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(logger, "Failed to start Xeno Control Guard Node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    // Shutdown ROS
    RCLCPP_INFO(logger, "Shutting down Xeno Control Guard Node");
    rclcpp::shutdown();
    return 0;
}