#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <functional>
#include <chrono>
#include <iostream>

#include <px4_msgs/msg/offboard_control_mode.hpp> // OffboardControlMode
#include <px4_msgs/msg/trajectory_setpoint.hpp> // target position, velocity, acc
#include <px4_msgs/msg/vehicle_command.hpp> // cmd like arm and disarm
#include <px4_msgs/msg/vehicle_control_mode.hpp> 

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class SLSqsf : public rclcpp::Node
{
public:
    SLSqsf()
    : Node("SLS_QSF") // name of the node
    {
        // publishers

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				// https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/10
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
        };
        timer_ = this->create_wall_timer(50ms, timer_callback); // 20Hz, define the spin rate of main loop
    }
    void arm();


private:
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void SLSqsf::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void SLSqsf::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1; // 1
	msg.param2 = param2; // 6
	msg.command = command; // VEHICLE_CMD_DO_SET_MODE = 176
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true; // command from ros2, not
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Disables buffering for standard output (stdout)
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SLSqsf>()); 

	rclcpp::shutdown();
	return 0;
}