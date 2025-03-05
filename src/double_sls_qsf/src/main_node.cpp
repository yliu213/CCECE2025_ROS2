// /****************************************************************************
//  *
//  * Copyright 2020 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice, this
//  * list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  * this list of conditions and the following disclaimer in the documentation
//  * and/or other materials provided with the distribution.
//  *
//  * 3. Neither the name of the copyright holder nor the names of its contributors
//  * may be used to endorse or promote products derived from this software without
//  * specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// /**
//  * @brief Offboard control example
//  * @file offboard_control.cpp
//  * @addtogroup examples
//  * @author Mickey Cowden <info@cowden.tech>
//  * @author Nuno Marques <nuno.marques@dronesolutions.io>
//  */

// #include <px4_msgs/msg/offboard_control_mode.hpp> // OffboardControlMode
// #include <px4_msgs/msg/trajectory_setpoint.hpp> // target position, velocity, acc
// #include <px4_msgs/msg/vehicle_command.hpp> // cmd like arm and disarm
// #include <px4_msgs/msg/vehicle_control_mode.hpp> 
// #include <rclcpp/rclcpp.hpp> // ROS2 C++ client library
// #include <stdint.h>

// #include <chrono> // timing utilities
// #include <iostream>

// using namespace std::chrono;
// using namespace std::chrono_literals;
// using namespace px4_msgs::msg;

// class QSFSLS : public rclcpp::Node
// {
// public:
// 	QSFSLS() : Node("offboard_control")
// 	{
// 		// publishers
// 		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
// 		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

// 		offboard_setpoint_counter_ = 0;

// 		auto timer_callback = [this]() -> void {

// 			if (offboard_setpoint_counter_ == 10) {
// 				// Change to Offboard mode after 10 setpoints
// 				// https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/10
// 				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

// 				// Arm the vehicle
// 				this->arm();
// 			}

// 			publish_offboard_control_mode();

// 			// stop the counter after reaching 11
// 			if (offboard_setpoint_counter_ < 11) {
// 				offboard_setpoint_counter_++;
// 			}
// 		};
// 		timer_ = this->create_wall_timer(50ms, timer_callback); // 20Hz, define the spin rate of main loop
// 	}
// 	void arm();

// private:
// 	rclcpp::TimerBase::SharedPtr timer_;

// 	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
// 	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

// 	uint64_t offboard_setpoint_counter_;

// 	void publish_offboard_control_mode();
// 	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
// };

// void QSFSLS::arm()
// {
// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
// 	RCLCPP_INFO(this->get_logger(), "Arm command send");
// }

// void QSFSLS::publish_offboard_control_mode()
// {
// 	OffboardControlMode msg{}; // define a msg object of type OffboardControlMode
// 	msg.position = true; // enable position control
// 	msg.velocity = false;
// 	msg.acceleration = false;
// 	msg.attitude = false;
// 	msg.body_rate = false;
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // get current time in microseconds
// 	offboard_control_mode_publisher_->publish(msg);
// }

// void QSFSLS::publish_vehicle_command(uint16_t command, float param1, float param2)
// {
// 	VehicleCommand msg{};
// 	msg.param1 = param1; // 1
// 	msg.param2 = param2; // 6
// 	msg.command = command; // VEHICLE_CMD_DO_SET_MODE = 176
// 	msg.target_system = 1;
// 	msg.target_component = 1;
// 	msg.source_system = 1;
// 	msg.source_component = 1;
// 	msg.from_external = true; // command from ros2, not
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	vehicle_command_publisher_->publish(msg);
// }

// int main(int argc, char *argv[])
// {
// 	std::cout << "Starting SLS control node" << std::endl;
// 	setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Disables buffering for standard output (stdout)
// 	rclcpp::init(argc, argv);
// 	rclcpp::spin(std::make_shared<QSFSLS>()); 

// 	rclcpp::shutdown();
// 	return 0;
// }
#include <px4_msgs/msg/offboard_control_mode.hpp> // OffboardControlMode
#include <px4_msgs/msg/trajectory_setpoint.hpp> // target position, velocity, acc
#include <px4_msgs/msg/vehicle_command.hpp> // cmd like arm and disarm
#include <px4_msgs/msg/vehicle_control_mode.hpp> 
#include <rclcpp/rclcpp.hpp> // ROS2 C++ client library
#include <stdint.h>

#include <chrono> // timing utilities
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// publishers
		// see .msg files for message type definitions
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				// https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/10
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback); // 10Hz, define the spin rate of main loop
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{}; // define a msg object of type OffboardControlMode
	msg.position = true; // enable position control
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // get current time in microseconds
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	// this part can be updated dynamically to some algorithm or even by a subscription callback for messages coming from another node
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0}; // NED frame, 5 meters above the ground
	msg.yaw = -3.14; // [-PI:PI], euler angle of desired yaw
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Disables buffering for standard output (stdout)
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>()); // 10Hz

	rclcpp::shutdown();
	return 0;
}
