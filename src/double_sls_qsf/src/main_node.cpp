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

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <px4_msgs/msg/offboard_control_mode.hpp> // OffboardControlMode
#include <px4_msgs/msg/trajectory_setpoint.hpp> // target position, velocity, acc
#include <px4_msgs/msg/vehicle_command.hpp> // cmd like arm and disarm
#include <px4_msgs/msg/vehicle_control_mode.hpp> 
#include <rclcpp/rclcpp.hpp> // ROS2 C++ client library
#include <stdint.h>

#include <chrono> // timing utilities
#include <iostream>

#include <gazebo_msgs/msg/link_states.hpp> // for gazebo link states
#include <Eigen/Dense>
#include "double_sls_qsf/common.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std::placeholders;

class SLSQSF : public rclcpp::Node
{
public:
	SLSQSF() : Node("SLS_QSF_controller")
	{
		// publishers
		// see .msg files for message type definitions
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		// subscribers
		gazebo_link_state_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states",1000,std::bind(&SLSQSF::gazeboLinkStateCb, this, _1));

		offboard_setpoint_counter_ = 0;
		Pos_ << 0.0, 0.0, 0.0; // current position
		Vel_ << 0.0, 0.0, 0.0; // current velocity

		RCLCPP_INFO(this->get_logger(),"Initialization Complete");
		init_complete_ = true;

		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				// https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/10
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

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

	// publihers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// subscribers
	rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_link_state_sub_;

	std::atomic<uint64_t> timestamp_;   // common synced timestamped

	// bools
	bool gazebo_link_name_matched_ = false;  // gazebo link indices
	bool init_complete_;

	// ints
	uint64_t offboard_setpoint_counter_;
	int uav_num_ = 1; // number of drones
    int uav_id_ = 1; // id of the drone
	int drone_link_index_;

	// vectors
	Eigen::Vector3d Pos_, loadPos_, pendAngle_, Vel_, mavRate_, loadVel_, pendRate_;
	Eigen::Vector4d Att_;

	// characters
	const char* gazebo_link_name_[1] = {
		"px4vision_sls::px4vision_ancl::base_link", 
	};

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void gazeboLinkStateCb(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void SLSQSF::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void SLSQSF::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SLSQSF::publish_offboard_control_mode()
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
void SLSQSF::publish_trajectory_setpoint()
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
void SLSQSF::publish_vehicle_command(uint16_t command, float param1, float param2)
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

void SLSQSF::gazeboLinkStateCb(const gazebo_msgs::msg::LinkStates::SharedPtr msg){
    if(!gazebo_link_name_matched_ && init_complete_){
        RCLCPP_INFO(this->get_logger(),"SLS gazebo"); 
        RCLCPP_INFO(this->get_logger(),"[gazeboLinkStateCb] Matching Gazebo Links");

        int n_name = sizeof(gazebo_link_name_)/sizeof(*gazebo_link_name_); 
        RCLCPP_INFO(this->get_logger(), "[gazeboLinkStateCb] n_name=%d", n_name);
        
		// for single SLS: 7 links per drone, remaming 2 for load and pendulum, 1 for ground_plane, 1 for asphault_plane
        int n_link = uav_num_*8+2+1;  
		RCLCPP_INFO(this->get_logger(), "[gazeboLinkStateCb] n_link=%d", n_link);
        int temp_index[n_name];
        for(int i=0; i<n_link; i++){
            for(int j=0; j<n_name; j++){
                if(msg->name[i] == gazebo_link_name_[j]){
                    temp_index[j] = i;
                };
            }
        }
        // note: the index can get through simulation in gazebo, or just use rostopic echo /gazebo/link_states/name to get
        drone_link_index_ = temp_index[uav_id_-1];
        gazebo_link_name_matched_ = true; 
		RCLCPP_INFO(this->get_logger(), "drone_link_index_=%d", drone_link_index_);
        RCLCPP_INFO(this->get_logger(), "[gazeboLinkStateCb] Matching Complete");
    }

    if(gazebo_link_name_matched_) {
        /* Get Gazebo Link States*/
        // >>> Pose
        // Drone
        Pos_ = toEigen(msg -> pose[drone_link_index_].position); // position
        Att_(0) = msg -> pose[drone_link_index_].orientation.w;
        Att_(1) = msg -> pose[drone_link_index_].orientation.x;
        Att_(2) = msg -> pose[drone_link_index_].orientation.y;
        Att_(3) = msg -> pose[drone_link_index_].orientation.z;    // orientation in quaternion form
        // Load 
        loadPos_ = toEigen(msg -> pose[10].position);  
        // Pendulum
        pendAngle_ = loadPos_ - Pos_;
        pendAngle_ = pendAngle_ / pendAngle_.norm();

        // >>> Velocity
        Vel_ = toEigen(msg -> twist[drone_link_index_].linear); //linear velocity
        mavRate_ = toEigen(msg -> twist[drone_link_index_].angular); // angular velocity
        loadVel_ = toEigen(msg -> twist[10].linear);
        pendRate_ = pendAngle_.cross(loadVel_ - Vel_);

        // since cmdloop never used, so remove the if() 
    	// diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
        // gazebo_last_called_ = ros::Time::now();

        //applyLowPassFilterFiniteDiff();
        //exeControl(); 
    }
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Disables buffering for standard output (stdout)
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SLSQSF>()); // 10Hz

	rclcpp::shutdown();
	return 0;
}
