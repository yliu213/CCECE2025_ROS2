// px4 msgs include
#include <px4_msgs/msg/offboard_control_mode.hpp> // OffboardControlMode
#include <px4_msgs/msg/trajectory_setpoint.hpp> // target position, velocity, acc
#include <px4_msgs/msg/vehicle_command.hpp> // cmd like arm and disarm
#include <px4_msgs/msg/vehicle_control_mode.hpp> 
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp> // attitude setpoint
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp> // body rate setpoint
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp> // thrust setpoint
#include "px4_ros_com/frame_transforms.h" // for frame transforms

// ros2 std lib include
#include <rclcpp/rclcpp.hpp> // ROS2 C++ client library
#include <stdint.h>
#include <cfloat> // for DBL_MIN
#include <chrono> // timing utilities
#include <iostream>
#include <rcl_interfaces/msg/set_parameters_result.hpp> // for setting parameters at runtime

// pkg related include
#include <gazebo_msgs/msg/link_states.hpp> // for gazebo link states
#include <Eigen/Dense>
#include "double_sls_qsf/common.h"
#include "double_sls_qsf/LowPassFilter.hpp"
#include "controller_msgs/msg/sls_state.hpp"
#include "controller_msgs/msg/sls_force.hpp"
#include "double_sls_qsf/QSFGeometricController.h"
#include "double_sls_qsf/nonlinear_attitude_control.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace controller_msgs::msg;
using namespace std::placeholders;

#define FD_EPSILON DBL_MIN

enum class ControllerPhase {
    WAIT_FOR_TAKEOFF,
    SLS_ENABLED
};

class SLSQSF : public rclcpp::Node
{
public:
	SLSQSF() : Node("SLS_QSF_controller")
	{
		/* Retrieve Parameters */
		// format: this->declare_parameter<type>("parameter_name", default_value);
		lpf_enabled_ = this->declare_parameter<bool>("lpf_enabled_", false);
		finite_diff_enabled_ = this->declare_parameter<bool>("finite_diff_enabled_", true);
        drag_comp_enabled_ = this->declare_parameter<bool>("drag_comp_enabled_", false);
        ctrl_enabled_ = this->declare_parameter<bool>("ctrl_enabled_", false);
        rate_ctrl_enabled_ = this->declare_parameter<bool>("rate_ctrl_enabled_", true);
        mission_enabled_ = this->declare_parameter<bool>("mission_enabled_", false);

		// Physical Parameters
		mass_ = this->declare_parameter<double>("mass_", 1.56);
		cable_length_ = this->declare_parameter<double>("cable_length_", 0.85);
    	load_mass_ = this->declare_parameter<double>("load_mass_", 0.25);

        // Initial Positions
        pos_x_0_ = this->declare_parameter<double>("pos_x_0_", 0.0);
        pos_y_0_ = this->declare_parameter<double>("pos_y_0_", 0.0);
        pos_z_0_ = this->declare_parameter<double>("pos_z_0_", 1.0);

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

        // Mission Setpoints
        c_x_1_ = this->declare_parameter<double>("c_x_1_", 0.0);
        c_y_1_ = this->declare_parameter<double>("c_y_1_", 0.0);
        c_z_1_ = this->declare_parameter<double>("c_z_1_", 1.0);
        c_x_2_ = this->declare_parameter<double>("c_x_2_", 0.0);
        c_y_2_ = this->declare_parameter<double>("c_y_2_", 0.0);
        c_z_2_ = this->declare_parameter<double>("c_z_2_", 1.0);
        c_x_3_ = this->declare_parameter<double>("c_x_3_", 0.0);
        c_y_3_ = this->declare_parameter<double>("c_y_3_", 0.0);
        c_z_3_ = this->declare_parameter<double>("c_z_3_", 1.0);

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

        // drone physical parameters
        max_fb_acc_ = this->declare_parameter<double>("max_fb_acc_", 9.0); //9.0

        // drone Yaw
        mavYaw_ = this->declare_parameter<double>("mavYaw_", 0.0);
        // attitude controller
        attctrl_tau = this->declare_parameter<double>("attctrl_tau", 0.3);

        // rotor drag compensation
        rotorDragD_x_ = this->declare_parameter<double>("rotorDragD_x_", 0.0);
        rotorDragD_y_ = this->declare_parameter<double>("rotorDragD_y_", 0.0);
        rotorDragD_z_ = this->declare_parameter<double>("rotorDragD_z_", 0.0);

        // throttle normalization
        norm_thrust_const_ = this->declare_parameter<double>("norm_thrust_const_", 0.05055);
        norm_thrust_offset_ = this->declare_parameter<double>("norm_thrust_offset_", 0.0);

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
        attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10); // for attitude control
        rate_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10); // for rate control
        //thrust_setpoint_publisher_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10); // for thrust control

		sls_state_raw_pub_ = this->create_publisher<SlsState> ("SLS_QSF_controller/sls_state_raw", 1);
		sls_state_pub_ = this->create_publisher<SlsState> ("SLS_QSF_controller/sls_state", 1);
		sls_force_pub_ = this->create_publisher<SlsForce> ("SLS_QSF_controller/sls_force", 1);

        // debug publisher
        rate_setpoint_debug_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/SLS_QSF_controller/debug_rate_sp", 10);
        attitude_setpoint_debug_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/SLS_QSF_controller/debug_att_sp", 10);

		// subscribers
		gazebo_link_state_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states",1000,std::bind(&SLSQSF::gazeboLinkStateCb, this, _1));

		offboard_setpoint_counter_ = 0;
		Pos_ << 0.0, 0.0, 0.0; // current position
		Vel_ << 0.0, 0.0, 0.0; // current velocity
		Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    	Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
        targetJerk_ = Eigen::Vector3d::Zero();

		targetPos_ << c_x_, c_y_, c_z_; 
		targetRadium_ << r_x_, r_y_, r_z_;
		targetFrequency_ << fr_x_, fr_y_, fr_z_;
		targetPhase_ << ph_x_, ph_y_, ph_z_;

        controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau); // initialize the ptr
        if (controller_) {
            RCLCPP_INFO(this->get_logger(), "controller_ is initialized");  // check if the controller is initialized
        } else {
            RCLCPP_WARN(this->get_logger(), "controller_ is nullptr!");
        }

		// initialization complete
		RCLCPP_INFO(this->get_logger(),"Initialization Complete");
		init_complete_ = true;
		gazebo_last_called_ = this->get_clock()->now();
        mission_last_called_ = this->get_clock()->now();

		// main loop
        auto timer_callback = [this]() -> void {
            switch (phase_) {
            case ControllerPhase::WAIT_FOR_TAKEOFF:
            {
                OffboardControlMode mode_msg{};
                mode_msg.position     = true;  
                mode_msg.velocity     = false;
                mode_msg.acceleration = false;
                mode_msg.attitude     = false;
                mode_msg.body_rate    = false;
                mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                offboard_control_mode_publisher_->publish(mode_msg);
        
                publish_trajectory_setpoint();  
        
                if (offboard_setpoint_counter_ == 10) {
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // OFFBOARD
                    arm();  
                    armed_time_ = this->get_clock()->now(); 
                }
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
        
                if (is_armed_) {
                    auto now = this->get_clock()->now();
                    double elapsed = (now - armed_time_).seconds();
                    if (elapsed > 7.0) {
                        RCLCPP_INFO(this->get_logger(), "7s after arming, switching to SLS controller");
                        phase_ = ControllerPhase::SLS_ENABLED;
                    }
                }
                break;
            }

            case ControllerPhase::SLS_ENABLED:
            {
                // Switch to body_rate offboard mode
                OffboardControlMode mode_msg{};
                mode_msg.position     = !(this-> ctrl_enabled_);
                mode_msg.velocity     = false;
                mode_msg.acceleration = false;
                mode_msg.attitude     = (this-> ctrl_enabled_) && (!(this-> rate_ctrl_enabled_));
                mode_msg.body_rate    = (this-> ctrl_enabled_) && (this-> rate_ctrl_enabled_);
                //mode_msg.thrust_and_torque = true;
                mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                offboard_control_mode_publisher_->publish(mode_msg);
                this-> test = true;
                
                break;
            }
            }
        };

        // parameter callback
        // this is needed to change the parameters at runtime using rqt_reconfigure
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            -> rcl_interfaces::msg::SetParametersResult
            {
              rcl_interfaces::msg::SetParametersResult result;
              result.successful = true;
              for (auto & param : params) {
                if (param.get_name() == "rate_ctrl_enabled_") {
                  rate_ctrl_enabled_ = param.as_bool();
                  RCLCPP_INFO(this->get_logger(), "Param changed: rate_ctrl_enabled=%s", rate_ctrl_enabled_ ? "true" : "false");
                } else if (param.get_name() == "mission_enabled_") {
                  mission_enabled_ = param.as_bool();
                  RCLCPP_INFO(this->get_logger(), "Param changed: mission_enabled=%s", mission_enabled_ ? "true" : "false");     
                } else if (param.get_name() == "ctrl_enabled_"){
                  ctrl_enabled_ = param.as_bool();
                  RCLCPP_INFO(this->get_logger(), "Param changed: ctrl_enabled=%s", ctrl_enabled_ ? "true" : "false");
                } else if (param.get_name() == "traj_tracking_enabled_"){
                  traj_tracking_enabled_ = param.as_bool();
                  RCLCPP_INFO(this->get_logger(), "Param changed: traj_tracking_enabled_=%s", traj_tracking_enabled_ ? "true" : "false");
                } else{}
              }
              return result;
            }
        );          
          
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
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_; // for attitude control
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rate_setpoint_publisher_; // for rate control
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rate_setpoint_debug_publisher_; // for debug
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_debug_publisher_; // for debug
    //rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_; // for thrust control

	// subscribers
	rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_link_state_sub_;

	// time related
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;   // common synced timestamped
	rclcpp::Time gazebo_last_called_;
	rclcpp::Time traj_tracking_last_called_;
    rclcpp::Time mission_last_called_;

    // takeoff related
    ControllerPhase phase_ = ControllerPhase::WAIT_FOR_TAKEOFF;
    rclcpp::Time armed_time_;   // to store when we armed
    bool is_armed_ = false;     // track if we’re armed
    bool test = false;

    // shared pointers
    std::shared_ptr<Control> controller_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; // for parameter callback

	// LPFS
	std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> load_vel_filter_;
	std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> vel_filter_;

	// bools
	bool gazebo_link_name_matched_ = false;  // gazebo link indices
	bool init_complete_;
	bool lpf_enabled_ = false;
    bool drag_comp_enabled_ = false;
    bool mission_enabled_ = false;
	bool finite_diff_enabled_;
	bool traj_tracking_enabled_, traj_tracking_enabled_last_;
    bool ctrl_enabled_;
    bool rate_ctrl_enabled_; // flag for rate control
    bool mission_initialized_ = false;

	// ints
	uint64_t offboard_setpoint_counter_;
	int uav_num_ = 1; // number of drones
    int uav_id_ = 1; // id of the drone
	int drone_link_index_;
    int mission_stage_ = 0;
    
	// doubles
	double diff_t_; // time difference
	double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_, Kacc_x_, Kacc_y_, Kacc_z_, Kjer_x_, Kjer_y_, Kjer_z_; // controller gains
	double load_mass_, cable_length_, mass_, max_fb_acc_;
	double gravity_acc_ = 9.80665;

	double c_x_, c_y_, c_z_;    
	double r_x_, r_y_, r_z_;   
	double fr_x_, fr_y_, fr_z_;
	double ph_x_, ph_y_, ph_z_; // reference trajectory parameters

    // mission setpoints
    double c_x_1_, c_y_1_, c_z_1_;
    double c_x_2_, c_y_2_, c_z_2_;
    double c_x_3_, c_y_3_, c_z_3_;

    double pos_x_0_, pos_y_0_, pos_z_0_;
    double mavYaw_; 
    double norm_thrust_const_, norm_thrust_offset_; // throttle normalization
    double attctrl_tau; // not defined in class in ros1 repo
    double rotorDragD_x_, rotorDragD_y_, rotorDragD_z_;

	// vectors
	Eigen::Vector3d Pos_, loadPos_, pendAngle_; // positon
	Eigen::Vector3d Pos_prev_, loadPos_prev_, pendAngle_prev_; // previous position

	Eigen::Vector3d Vel_, Rate_, loadVel_, pendRate_; // velocity
	Eigen::Vector3d Vel_prev_, Rate_prev_, loadVel_prev_, pendRate_prev_; // previous velocity

	Eigen::Vector3d loadAcc_, pendAngularAcc_; // acceleration
	Eigen::Vector3d loadAcc_prev_, pendAngularAcc_prev_; // previous acceleration

	Eigen::Vector4d Att_, Att_prev_, q_des_; // orientation	
    Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}

	Eigen::Vector3d targetRadium_, targetFrequency_, targetPhase_; // reference trajectory parameters
	Eigen::Vector3d targetPos_; // control target position

    Eigen::Vector3d targetJerk_;
	Eigen::Vector3d Kpos_, Kvel_;
    Eigen::Vector3d rotorDragD_;

    Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.80665)};

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
    Eigen::Vector3d compensateRotorDrag(double t);
    Eigen::Vector3d transformPose(Eigen::Vector3d oldPose, Eigen::Vector3d offsetVector);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
    void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
    void updateReference();
    void checkMissionStage(double mission_time_span);
    void debugRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
};

void SLSQSF::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
    is_armed_ = true;
    armed_time_ = this->get_clock()->now(); // store the time
}

void SLSQSF::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void SLSQSF::publish_offboard_control_mode()
{
	OffboardControlMode msg{}; // define a msg object of type OffboardControlMode
	msg.position = true; // enable position control
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false; // enable attitude control
	msg.body_rate = false; // enable body rate control
	msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
	offboard_control_mode_publisher_->publish(msg);
}

void SLSQSF::publish_trajectory_setpoint()
{
	// this part can be updated dynamically to some algorithm or even by a subscription callback for messages coming from another node
	TrajectorySetpoint msg{};
	//msg.position = {0.0, -1.0, -1.0}; // gazebo: x:-1, y:0, z:1
    //msg.position = {-1.0, 0.0, -1.0}; // gazebo: x:0, y:-1, z:1
    msg.position = {0.0, 0.0, -1.0};
	// msg.yaw = 3.14159265358979323846/2; 
    msg.yaw = 0.0; 
	msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
	trajectory_setpoint_publisher_->publish(msg);
}

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
	msg.timestamp = this->get_clock()->now().seconds();
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
    	diff_t_ = this->get_clock()->now().seconds() - gazebo_last_called_.seconds();
        gazebo_last_called_ = this->get_clock()->now();

        applyLowPassFilterFiniteDiff();
        exeControl(); 
    }
}

void SLSQSF::exeControl(void){
        //RCLCPP_INFO(this->get_logger(),"SLS Control EXE");
        if(init_complete_){
            if(traj_tracking_enabled_ && !traj_tracking_enabled_last_) {
                traj_tracking_last_called_ = this->get_clock()->now();
            }
            traj_tracking_enabled_last_ = traj_tracking_enabled_;

            Eigen::Vector3d desired_acc;
            desired_acc = applyQuasiSlsCtrl();
            computeBodyRateCmd(cmdBodyRate_, desired_acc);
            if(ctrl_enabled_ && this-> test){
                pubRateCommands(cmdBodyRate_, q_des_); 
            }
            else{
                publish_trajectory_setpoint(); // px4 default control
                debugRateCommands(cmdBodyRate_, q_des_);
            }
            updateReference();
        }
}

void SLSQSF::updateReference(){
    double t = this->get_clock()->now().seconds();
    double sp_x = c_x_ + r_x_ * std::sin(fr_x_ * t + ph_x_);
    double sp_y = c_y_ + r_y_ * std::sin(fr_y_ * t + ph_y_);
    double sp_z = c_z_ + r_z_ * std::sin(fr_z_ * t + ph_z_);
    double sp_x_dt = r_x_ * fr_x_ * std::cos(fr_x_ * t + ph_x_);
    double sp_y_dt = r_y_ * fr_y_ * std::cos(fr_y_ * t + ph_y_);
    double sp_z_dt = r_z_ * fr_z_ * std::cos(fr_z_ * t + ph_z_);

    if(mission_enabled_){
        switch(mission_stage_){
            case 0:
                if(!mission_initialized_){
                    RCLCPP_INFO(this->get_logger(),"[exeMission] Mission started at case 0");
                    mission_initialized_ = true;
                }
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 1:
                targetPos_ << c_x_1_, c_y_1_, c_z_1_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 2:
                targetPos_ << c_x_2_, c_y_2_, c_z_2_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 3:
                targetPos_ << c_x_3_, c_y_3_, c_z_3_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 4:
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 5:
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << r_x_, r_y_, r_z_;
                targetFrequency_ << fr_x_, fr_y_, fr_z_;
                targetPhase_ << ph_x_, ph_y_, ph_z_; 
                this->traj_tracking_enabled_ = true; 
                checkMissionStage(20);
                break;
            default:
                targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                this->traj_tracking_enabled_ = false;
                if(this->get_clock()->now().seconds() - mission_last_called_.seconds() >= 10){
                    RCLCPP_INFO(this->get_logger(),"[exeMission] Mission Accomplished");
                    mission_last_called_ = this->get_clock()->now();
                    this->mission_initialized_ = false;
                }
        }
    }
    else {
        mission_last_called_ = this->get_clock()->now();
        if(mission_stage_ == 5) {
            this->traj_tracking_enabled_ = false;
        }
        mission_stage_ = 0;
        mission_initialized_ = false;
        targetPos_ << c_x_, c_y_, c_z_; 
        targetRadium_ << r_x_, r_y_, r_z_;
        targetFrequency_ << fr_x_, fr_y_, fr_z_;
        targetPhase_ << ph_x_, ph_y_, ph_z_;
    }
}

void SLSQSF::checkMissionStage(double mission_time_span) {
    if(this->get_clock()->now().seconds() - mission_last_called_.seconds() >= mission_time_span) {
        mission_last_called_ = this->get_clock()->now();
        mission_stage_ += 1;
        RCLCPP_INFO(this->get_logger(),"[exeMission] Stage %d ended, switching to stage %d", mission_stage_ - 1, mission_stage_);
    }
}

void SLSQSF::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    // ros2 equivalent
    if(rate_ctrl_enabled_){
        VehicleRatesSetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.roll = static_cast<float>(cmd(0));
        msg.pitch = static_cast<float>(cmd(1));
        msg.yaw = static_cast<float>(cmd(2));
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(-cmd(3)); 
        rate_setpoint_publisher_ -> publish(msg);
    } else {
        VehicleAttitudeSetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000; 
        // transform from ENU
        Eigen::Quaterniond target_att_enu(target_attitude(0), target_attitude(1), target_attitude(2), target_attitude(3));
        Eigen::Quaterniond target_att_ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(target_att_enu);
        msg.q_d[0] = static_cast<float>(target_att_ned.w());
        msg.q_d[1] = static_cast<float>(target_att_ned.x());
        msg.q_d[2] = static_cast<float>(target_att_ned.y());
        msg.q_d[3] = static_cast<float>(target_att_ned.z());

        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(-cmd(3));
        attitude_setpoint_publisher_ -> publish(msg);
    }
}

void SLSQSF::debugRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    // ros2 equivalent
    if(rate_ctrl_enabled_){
        VehicleRatesSetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.roll = static_cast<float>(cmd(0));
        msg.pitch = static_cast<float>(cmd(1));
        msg.yaw = static_cast<float>(cmd(2));
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(-cmd(3)); 
        rate_setpoint_debug_publisher_ -> publish(msg);
    } else {
        VehicleAttitudeSetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.q_d[0] = static_cast<float>(target_attitude(0)); // w
        msg.q_d[1] = static_cast<float>(target_attitude(1)); // x
        msg.q_d[2] = static_cast<float>(target_attitude(2)); // y
        msg.q_d[3] = static_cast<float>(target_attitude(3)); // z
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = cmd(3); 
        attitude_setpoint_debug_publisher_ -> publish(msg);
    }
}

void SLSQSF::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
    q_des_ = acc2quaternion(a_des, mavYaw_);
    controller_ -> Update(Att_, q_des_, a_des, targetJerk_); 
    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();
    bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command + norm_thrust_offset_));  // original, seems normalized to 0-1
}

Eigen::Vector3d SLSQSF::applyQuasiSlsCtrl(){
    double target_force_ned[3];
    double K[10] = {Kpos_x_, Kvel_x_, Kacc_x_, Kjer_x_, Kpos_y_, Kvel_y_, Kacc_y_, Kjer_y_, Kpos_z_, Kvel_z_};
    double param[4] = {load_mass_, mass_, cable_length_, gravity_acc_}; 
    double ref[12] = {
        targetRadium_(1), targetFrequency_(1), targetPos_(1), targetPhase_(1), 
        targetRadium_(0), targetFrequency_(0), targetPos_(0), targetPhase_(0), // convert to NED for the controller
        targetRadium_(2), targetFrequency_(2), -targetPos_(2), targetPhase_(2)}; 
    if(!traj_tracking_enabled_) {
        for(int i=0; i<12; i++){
            if((i+2)%4!=0) ref[i]=0; // set all ref to 0 except for the position
        }
    }
    double sls_state_array[12];

    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_state[i]; 
    }
    const double t = this->get_clock()->now().seconds() - traj_tracking_last_called_.seconds(); 
    QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned);

    sls_force_.header.stamp = this->get_clock()->now();
    sls_force_.sls_force[0] = target_force_ned[0];
    sls_force_.sls_force[1] = target_force_ned[1];
    sls_force_.sls_force[2] = target_force_ned[2];
    sls_force_pub_ -> publish(sls_force_);

    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[1] / mass_;
    a_des(1) = target_force_ned[0] / mass_;
    a_des(2) = -target_force_ned[2] / mass_; // seems back to ENU for a_des

    // a_des(0) = target_force_ned[0] / mass_;
    // a_des(1) = target_force_ned[1] / mass_;
    // a_des(2) = target_force_ned[2] / mass_;  // still in NED for a_des

    Eigen::Vector3d a_fb = a_des + gravity_;

    if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; 

    // rotor drag compensation
    Eigen::Vector3d a_rd;
    if(drag_comp_enabled_) {
        a_rd = compensateRotorDrag(t);
    }
    else {
        a_rd = Eigen::Vector3d::Zero();
    } 

    a_des = a_fb - a_rd - gravity_; // seems from (31)
    
    // to NED
    // Eigen::Vector3d a_des_ned;
    // a_des_ned(0) = a_des(1);
    // a_des_ned(1) = a_des(0);
    // a_des_ned(2) = -a_des(2);
    // return a_des_ned;

    // this seems to work, but wrong
    // a_des(1) = a_des(0);
    // a_des(0) = a_des(1);
    // a_des(2) = -a_des(2);

    return a_des; 
}

Eigen::Vector3d SLSQSF::compensateRotorDrag(double t) {
    bool use_feedback = true;
    // mav vel ref
    Eigen::Vector3d loadVelRDC, loadAccRDC, loadVelFF, loadAccFF, loadVelFB, loadAccFB;
    loadVelFF(0) = targetRadium_(0) * targetFrequency_(0) * std::cos(targetFrequency_(0) * t + targetPhase_(0));
    loadVelFF(1) = targetRadium_(1) * targetFrequency_(1) * std::cos(targetFrequency_(1) * t + targetPhase_(1));
    loadVelFF(2) = targetRadium_(2) * targetFrequency_(2) * std::cos(targetFrequency_(2) * t + targetPhase_(2));
    loadAccFF(0) = -targetRadium_(0) * std::pow(targetFrequency_(0), 2) * std::sin(targetFrequency_(0) * t + targetPhase_(0));
    loadAccFF(1) = -targetRadium_(1) * std::pow(targetFrequency_(1), 2) * std::sin(targetFrequency_(1) * t + targetPhase_(1));
    loadAccFF(2) = -targetRadium_(2) * std::pow(targetFrequency_(2), 2) * std::sin(targetFrequency_(2) * t + targetPhase_(2));
    loadVelFB = loadVel_;
    loadAccFB = loadAcc_;

    if(traj_tracking_enabled_) {
        loadVelRDC = loadVelFF;
        loadAccRDC = loadAccFF;
    }
    else if(use_feedback) {
        loadVelRDC = loadVelFB;
        loadAccRDC = loadAccFB;
    }
    else {
        loadVelRDC = Eigen::Vector3d::Zero();
        loadAccRDC = Eigen::Vector3d::Zero();
    }
    
    const Eigen::Vector3d mavVelRef = transformPose(loadVelRDC, -pendRate_);
    const Eigen::Vector3d mavAccRef = transformPose(loadAccRDC, -pendAngularAcc_);

    // mav RotMat ref
    const Eigen::Vector4d q_ref = acc2quaternion(mavAccRef - gravity_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    // drag coefficient
    rotorDragD_ << rotorDragD_x_, rotorDragD_y_, rotorDragD_z_;

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = -R_ref * rotorDragD_.asDiagonal() * R_ref.transpose() * mavVelRef;  // Rotor drag

    return a_rd;
}

Eigen::Vector4d SLSQSF::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    // copied from korean repo
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des, proj_yb_des;
    Eigen::Matrix3d rotmat;

    // ENU
    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0; // (9)
    zb_des = vector_acc / vector_acc.norm(); // (33)
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm(); // (34)
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm(); // (35)

    // NED
    // proj_yb_des << -std::sin(yaw), std::cos(yaw), 0.0; 
    // zb_des = -vector_acc / vector_acc.norm(); 
    // xb_des = zb_des.cross(proj_yb_des) / (zb_des.cross(proj_yb_des)).norm(); 
    // yb_des = xb_des.cross(zb_des) / (xb_des.cross(zb_des)).norm(); 

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);

    return quat;
}

Eigen::Vector3d SLSQSF::transformPose(Eigen::Vector3d oldPose, Eigen::Vector3d offsetVector) {
    Eigen::Vector3d newPose;
    Eigen::Vector4d oldPose4, newPose4;
    Eigen::Matrix4d transMatrix;

    // >>> Translation Matrix
    transMatrix <<  1, 0, 0, offsetVector(0),
                    0, 1, 0, offsetVector(1),
                    0, 0, 1, offsetVector(2), 
                    0, 0, 0, 1;

    // >>> old pose
    oldPose4.head(3) = oldPose;
    oldPose4(3) = 1;

    // >>> new pose
    newPose4 = transMatrix*oldPose4;
    newPose = newPose4.head(3);

    return newPose;
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
            sls_state_raw_.sls_state[5] = -pendAngle_(2); // store value in NED frame
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
                sls_state_raw_.sls_state[8] = -loadVel_(2); // store value in NED frame
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
