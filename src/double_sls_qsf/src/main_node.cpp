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

#include <cfloat> // for DBL_MIN
#include <chrono> // timing utilities
#include <iostream>

#include <gazebo_msgs/msg/link_states.hpp> // for gazebo link states
#include <Eigen/Dense>
#include "double_sls_qsf/common.h"
#include "double_sls_qsf/LowPassFilter.hpp"
#include "controller_msgs/msg/sls_state.hpp"
#include "controller_msgs/msg/sls_force.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace controller_msgs::msg;
using namespace std::placeholders;

#define FD_EPSILON DBL_MIN

class SLSQSF : public rclcpp::Node
{
public:
	SLSQSF() : Node("SLS_QSF_controller")
	{
		/* Retrieve Parameters */
		// format: this->declare_parameter<type>("parameter_name", default_value);
		lpf_enabled_ = this->declare_parameter<bool>("lpf_enabled_", false);
		finite_diff_enabled_ = this->declare_parameter<bool>("finite_diff_enabled_", true);

		// Physical Parameters
		mass_ = this->declare_parameter<double>("mass_", 1.56);
		cable_length_ = this->declare_parameter<double>("cable_length_", 0.85);
    	load_mass_ = this->declare_parameter<double>("load_mass_", 0.25);

		// Controller Gains
		Kpos_x_ = this->declare_parameter<double>("Kpos_x_", 10.0);
		Kpos_y_ = this->declare_parameter<double>("Kpos_y_", 10.0);
		Kpos_z_ = this->declare_parameter<double>("Kpos_z_", 20.0);
		Kvel_x_ = this->declare_parameter<double>("Kvel_x_", 5.0);
		Kvel_y_ = this->declare_parameter<double>("Kvel_y_", 5.0);
		Kvel_z_ = this->declare_parameter<double>("Kvel_z_", 10.0);
		Kacc_x_ = this->declare_parameter<double>("Kacc_x_", 0.0);
		Kacc_y_ = this->declare_parameter<double>("Kacc_y_", 0.0);
		Kacc_z_ = this->declare_parameter<double>("Kacc_z_", 0.0);
		Kjer_x_ = this->declare_parameter<double>("Kjer_x_", 0.0);
		Kjer_y_ = this->declare_parameter<double>("Kjer_y_", 0.0);
		Kjer_z_ = this->declare_parameter<double>("Kjer_z_", 0.0); 

		// Reference
		c_x_ = this->declare_parameter<double>("c_x_", 0.0);
		c_y_ = this->declare_parameter<double>("c_y_", 0.0);
		c_z_ = this->declare_parameter<double>("c_z_", 1.0);    
		r_x_ = this->declare_parameter<double>("r_x_", 0.0);
		r_y_ = this->declare_parameter<double>("r_y_", 0.0);    
		r_z_ = this->declare_parameter<double>("r_z_", 0.0);  
		fr_x_ = this->declare_parameter<double>("fr_x_",0.0);
		fr_y_ = this->declare_parameter<double>("fr_y_", 0.0);    
		fr_z_ = this->declare_parameter<double>("fr_z_", 0.0);      
		ph_x_ = this->declare_parameter<double>("ph_x_", 0.0);
		ph_y_ = this->declare_parameter<double>("ph_y_", 0.0);    
		ph_z_ = this->declare_parameter<double>("ph_z_", 0.0); 

		// LPFs
		double load_vel_cutoff_freq = this->declare_parameter<double>("load_vel_cutoff_freq", 30);
		double load_vel_q = this->declare_parameter<double>("load_vel_q", 0.625);
		bool load_vel_verbose = this->declare_parameter<bool>("load_vel_verbose", false);
		load_vel_filter_ = std::make_unique<SecondOrderFilter<Eigen::Vector3d>>(load_vel_cutoff_freq, load_vel_q, load_vel_verbose);

		double mav_vel_cutoff_freq = this->declare_parameter("mav_vel_cutoff_freq", 30);
		double mav_vel_q = this->declare_parameter<double>("mav_vel_q", 0.625);
		bool mav_vel_verbose = this->declare_parameter<bool>("mav_vel_verbose", false);
		vel_filter_ = std::make_unique<SecondOrderFilter<Eigen::Vector3d>>(mav_vel_cutoff_freq, mav_vel_q, mav_vel_verbose);
	
		// publishers
		// see .msg files for message type definitions
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		sls_state_raw_pub_ = this->create_publisher<SlsState> ("SLS_QSF_controller/sls_state_raw", 1);
		sls_state_pub_ = this->create_publisher<SlsState> ("SLS_QSF_controller/sls_state", 1);
		sls_force_pub_ = this->create_publisher<SlsForce> ("SLS_QSF_controller/sls_force", 1);

		// subscribers
		gazebo_link_state_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states",1000,std::bind(&SLSQSF::gazeboLinkStateCb, this, _1));

		offboard_setpoint_counter_ = 0;
		Pos_ << 0.0, 0.0, 0.0; // current position
		Vel_ << 0.0, 0.0, 0.0; // current velocity
		Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    	Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

		targetPos_ << c_x_, c_y_, c_z_; 
		targetRadium_ << r_x_, r_y_, r_z_;
		targetFrequency_ << fr_x_, fr_y_, fr_z_;
		targetPhase_ << ph_x_, ph_y_, ph_z_;

		// initialization complete
		RCLCPP_INFO(this->get_logger(),"Initialization Complete");
		init_complete_ = true;
		gazebo_last_called_ = this->get_clock()->now();

		// main loop
		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				// https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/10
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(50ms, timer_callback); // 20Hz
	}

	void arm();
	void disarm();

private:
	// publihers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<SlsState>::SharedPtr sls_state_raw_pub_;
	rclcpp::Publisher<SlsState>::SharedPtr sls_state_pub_;
	rclcpp::Publisher<SlsForce>::SharedPtr sls_force_pub_;

	// subscribers
	rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_link_state_sub_;

	// time related
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;   // common synced timestamped
	rclcpp::Time gazebo_last_called_;
	rclcpp::Time traj_tracking_last_called_;

	// LPFS
	std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> load_vel_filter_;
	std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> vel_filter_;

	// bools
	bool gazebo_link_name_matched_ = false;  // gazebo link indices
	bool init_complete_;
	bool lpf_enabled_ = false;
	bool finite_diff_enabled_;
	bool traj_tracking_enabled_, traj_tracking_enabled_last_;

	// ints
	uint64_t offboard_setpoint_counter_;
	int uav_num_ = 1; // number of drones
    int uav_id_ = 1; // id of the drone
	int drone_link_index_;

	// doubles
	double diff_t_; // time difference
	double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_, Kacc_x_, Kacc_y_, Kacc_z_, Kjer_x_, Kjer_y_, Kjer_z_; // controller gains
	double load_mass_, cable_length_, mass_;
	double gravity_acc_ = 9.80665;

	double c_x_, c_y_, c_z_;    
	double r_x_, r_y_, r_z_;   
	double fr_x_, fr_y_, fr_z_;
	double ph_x_, ph_y_, ph_z_; // reference trajectory parameters

	// vectors
	Eigen::Vector3d Pos_, loadPos_, pendAngle_; // positon
	Eigen::Vector3d Pos_prev_, loadPos_prev_, pendAngle_prev_; // previous position

	Eigen::Vector3d Vel_, Rate_, loadVel_, pendRate_; // velocity
	Eigen::Vector3d Vel_prev_, Rate_prev_, loadVel_prev_, pendRate_prev_; // previous velocity

	Eigen::Vector3d loadAcc_, pendAngularAcc_; // acceleration
	Eigen::Vector3d loadAcc_prev_, pendAngularAcc_prev_; // previous acceleration

	Eigen::Vector4d Att_, Att_prev_; // orientation	

	Eigen::Vector3d targetRadium_, targetFrequency_, targetPhase_; // reference trajectory parameters
	Eigen::Vector3d targetPos_; // control target position

	Eigen::Vector3d Kpos_, Kvel_;

	// controller msgs
	SlsState sls_state_raw_; 
    SlsState sls_state_; 
    SlsForce sls_force_; 

	// characters
	const char* gazebo_link_name_[1] = {
		"px4vision_sls::px4vision_ancl::base_link", 
	};

	// functions
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void gazeboLinkStateCb(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
	void applyLowPassFilterFiniteDiff(void);
	void applyIteration(void);
	void loadSlsState();
	void exeControl(void);
	Eigen::Vector3d applyQuasiSlsCtrl();
};


void SLSQSF::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

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
        Rate_ = toEigen(msg -> twist[drone_link_index_].angular); // angular velocity
        loadVel_ = toEigen(msg -> twist[10].linear);
        pendRate_ = pendAngle_.cross(loadVel_ - Vel_);

        // since cmdloop never used, so remove the if()
    	diff_t_ = (this->get_clock()->now() - gazebo_last_called_).seconds();
        gazebo_last_called_ = this->get_clock()->now();

        applyLowPassFilterFiniteDiff();
        exeControl(); 
    }
}

// in progress
void SLSQSF::exeControl(void){
    RCLCPP_INFO(this->get_logger(),"SLS Control EXE");
    if(init_complete_){
        if(traj_tracking_enabled_ && !traj_tracking_enabled_last_) {
            traj_tracking_last_called_ = this->get_clock()->now();
        }
        traj_tracking_enabled_last_ = traj_tracking_enabled_;

        Eigen::Vector3d desired_acc;
        desired_acc = applyQuasiSlsCtrl(); // <- in progress
    //     computeBodyRateCmd(cmdBodyRate_, desired_acc);
    //     if(ctrl_enabled_){
    //         pubRateCommands(cmdBodyRate_, q_des_); 
    //     }
    //     else{
    //         // use px4's position controller
    //         pubTargetPose(pos_x_0_, pos_y_0_, pos_z_0_);
    //         debugRateCommands(cmdBodyRate_, q_des_); // same as pubRateCommands
    //     }
    //     updateReference();
    }
}

// in progress
Eigen::Vector3d SLSQSF::applyQuasiSlsCtrl(){
    double target_force_ned[3];
    double K[10] = {Kpos_x_, Kvel_x_, Kacc_x_, Kjer_x_, Kpos_y_, Kvel_y_, Kacc_y_, Kjer_y_, Kpos_z_, Kvel_z_};
    double param[4] = {load_mass_, mass_, cable_length_, gravity_acc_}; // some physical parameters
    double ref[12] = {
        targetRadium_(1), targetFrequency_(1), targetPos_(1), targetPhase_(1), 
        targetRadium_(0), targetFrequency_(0), targetPos_(0), targetPhase_(0), // convert to NED
        targetRadium_(2), targetFrequency_(2), -targetPos_(2), targetPhase_(2)}; // reference trajectory parameters
    if(!traj_tracking_enabled_) {
        for(int i=0; i<12; i++){
            if((i+2)%4!=0) ref[i]=0; // set all ref to 0 except for the position
        }
    }
    double sls_state_array[12];

    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_state[i]; 
    }
    const double t = (this->get_clock()->now() - traj_tracking_last_called_).seconds();
    // QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned); // generated from MATLAB

    // // if(std::abs(target_force_ned[0]) > 2 ) {
    // //     target_force_ned[0] = std::copysign(2, target_force_ned[0]);
    // // }

    // // if(std::abs(target_force_ned[1]) > 2 ) {
    // //     target_force_ned[1] = std::copysign(2,target_force_ned[1]);
    // // }

    // sls_force_.header.stamp = ros::Time::now();
    // sls_force_.sls_force[0] = target_force_ned[0];
    // sls_force_.sls_force[1] = target_force_ned[1];
    // sls_force_.sls_force[2] = target_force_ned[2];
    // sls_force_pub_.publish(sls_force_);
    // // ROS_INFO_STREAM("SLS Force: " << target_force_ned[2]);
    // Eigen::Vector3d a_des;
    // a_des(0) = target_force_ned[1] / mass_;
    // a_des(1) = target_force_ned[0] / mass_;
    // a_des(2) = -target_force_ned[2] / mass_;

    // Eigen::Vector3d a_fb = a_des + gravity_;

    // if (a_fb.norm() > max_fb_acc_)
    // a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; 

    // // rotor drag compensation
    // Eigen::Vector3d a_rd;
    // if(drag_comp_enabled_) {
    //     a_rd = compensateRotorDrag(t);
    // }
    // else {
    //     a_rd = Eigen::Vector3d::Zero();
    // } 

    // a_des = a_fb - a_rd - gravity_;
    
    // return a_des;
	return Eigen::Vector3d::Zero(); // for test only, remove later
}

void SLSQSF::applyLowPassFilterFiniteDiff(void) {
    // compute quaternion error
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0); 
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * Att_prev_;  
    const Eigen::Vector4d qe = quatMultiplication(q_inv, Att_); 

    if(lpf_enabled_) {
        pendAngle_ = loadPos_ - Pos_;
        // >>> Finite Diff
        if(finite_diff_enabled_) {
            sls_state_raw_.sls_state[0] = loadPos_(1);
            sls_state_raw_.sls_state[1] = loadPos_(0);
            sls_state_raw_.sls_state[2] = -loadPos_(2);
            sls_state_raw_.sls_state[3] = pendAngle_(1);
            sls_state_raw_.sls_state[4] = pendAngle_(0);
            sls_state_raw_.sls_state[5] = -pendAngle_(2);
            if(diff_t_ > FD_EPSILON) {
                // >>> mavVel
                Vel_ = (Pos_ - Pos_prev_) / diff_t_;
                Vel_ = vel_filter_ -> updateFilter(Vel_, diff_t_);
                // >>> mavRate
                Rate_(0) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(1);
                Rate_(1) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(2);
                Rate_(2) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(3);
                // >>> loadVel
                loadVel_ = (loadPos_ - loadPos_prev_) / diff_t_;
                sls_state_raw_.sls_state[6] = loadVel_(1);
                sls_state_raw_.sls_state[7] = loadVel_(0);
                sls_state_raw_.sls_state[8] = -loadVel_(2); 
                loadVel_ = load_vel_filter_ -> updateFilter(loadVel_, diff_t_);
                // >>> loadAcc
                loadAcc_ = (loadVel_ - loadVel_prev_) / diff_t_;
                // TODO loadAcc_ = load_acc_filter ->
                // >>> pendRate
                pendRate_ = pendAngle_.cross(loadVel_ - Vel_);
                sls_state_raw_.sls_state[9] = pendRate_(1);
                sls_state_raw_.sls_state[10] = pendRate_(0);
                sls_state_raw_.sls_state[11] = -pendRate_(2);
                // pendRate_ = pendAngle_.cross((pendAngle_ - pendAngle_prev_) / diff_t_);
                // pendRate_ = pend_rate_filter_ -> updateFilter(pendRate_, diff_t_);
                // >>> pendAngularAcc
                pendAngularAcc_ = (pendRate_ - pendRate_prev_) / diff_t_;
                // TODO pendAngularAcc_ = pend_angular_acc_filter ->


                applyIteration();
            }
            else {
                Vel_ = Vel_prev_;
                Rate_ = Rate_prev_;
                loadVel_ = loadVel_prev_;
                loadAcc_ = loadAcc_prev_;
                pendRate_ = pendRate_prev_;
                pendAngularAcc_ = pendAngularAcc_prev_;
                sls_state_raw_.sls_state[6] = loadVel_(1);
                sls_state_raw_.sls_state[7] = loadVel_(0);
                sls_state_raw_.sls_state[8] = -loadVel_(2); 
                sls_state_raw_.sls_state[9] = pendRate_(1);
                sls_state_raw_.sls_state[10] = pendRate_(0);
                sls_state_raw_.sls_state[11] = -pendRate_(2);
            }
            sls_state_raw_.header.stamp = this->get_clock()->now();
            sls_state_raw_pub_->publish(sls_state_raw_); 

        }

        else {
            RCLCPP_INFO(this->get_logger(),"Error: Finite Difference Not Enabled when LPF is called!");
        }

        // >>> publish filtered data
        loadSlsState();
        sls_state_pub_->publish(sls_state_); 
    }

    else { // LPF not enabled
        // pendAngle
        pendAngle_ = loadPos_ - Pos_;
        pendAngle_ = pendAngle_ / pendAngle_.norm();

        if(finite_diff_enabled_) {
            if(diff_t_ > FD_EPSILON) { //make sure diff_t_ > smallest positive value
                // >>> mavVel
                Vel_ = (Pos_ - Pos_prev_) / diff_t_;
                // >>> mavRate
                Rate_(0) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(1);
                Rate_(1) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(2);
                Rate_(2) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(3);
                // >>> loadVel
                loadVel_ = (loadPos_ - loadPos_prev_) / diff_t_;
                // >>> loadAcc
                loadAcc_ = (loadVel_ - loadVel_prev_) / diff_t_;
                // >>> pendRate
                pendRate_ = pendAngle_.cross((pendAngle_ - pendAngle_prev_) / diff_t_);
                // >>> pendAngularAcc
                pendAngularAcc_ = (pendRate_ - pendRate_prev_) / diff_t_;
                applyIteration();
            }
            else {
                Vel_ = Vel_prev_;
                Rate_ = Rate_prev_;
                loadVel_ = loadVel_prev_;
                loadAcc_ = loadAcc_prev_;
                pendRate_ = pendRate_prev_;
                pendAngularAcc_ = pendAngularAcc_prev_;
            }

            // publish finite difference results
            loadSlsState();
            sls_state_pub_ -> publish(sls_state_); 
        }

        else {
            RCLCPP_INFO(this->get_logger(),"Error: Finite Difference Not Enabled when LPF is called!");
        }
    }
}

void SLSQSF::applyIteration(void) {
    /* Iteration */
    Pos_prev_ = Pos_;
    Vel_prev_ = Vel_;
    Att_prev_ = Att_;
    Rate_prev_ = Rate_;
    loadPos_prev_ = loadPos_;
    loadVel_prev_ = loadVel_;
    loadAcc_prev_ = loadAcc_;
    pendAngle_prev_ = pendAngle_;
    pendRate_prev_ = pendRate_;
    pendAngularAcc_prev_ = pendAngularAcc_;
}

void SLSQSF::loadSlsState() {
    // the paper uses NED frame, but the gazebo uses ENU frame
    // state vector: [xp, w, dxp, dw]
    sls_state_.header.stamp = this->get_clock()->now();
    sls_state_.sls_state[0] = loadPos_(1);
    sls_state_.sls_state[1] = loadPos_(0);
    sls_state_.sls_state[2] = -loadPos_(2);
    sls_state_.sls_state[3] = pendAngle_(1);
    sls_state_.sls_state[4] = pendAngle_(0);
    sls_state_.sls_state[5] = -pendAngle_(2);
    sls_state_.sls_state[6] = loadVel_(1);
    sls_state_.sls_state[7] = loadVel_(0);
    sls_state_.sls_state[8] = -loadVel_(2); 
    sls_state_.sls_state[9] = pendRate_(1);
    sls_state_.sls_state[10] = pendRate_(0);
    sls_state_.sls_state[11] = -pendRate_(2);
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
