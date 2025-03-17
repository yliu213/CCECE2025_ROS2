/****************************************************************************
 *
 *   Copyright (c) 2018-2022 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Controller base class
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

 #include "double_sls_qsf/nonlinear_attitude_control.h"
 #include "px4_ros_com/frame_transforms.h" // for frame transforms
 #include <rclcpp/rclcpp.hpp>

 NonlinearAttitudeControl::NonlinearAttitudeControl(double attctrl_tau) : Control() { attctrl_tau_ = attctrl_tau; }
 
 NonlinearAttitudeControl::~NonlinearAttitudeControl() {}
 
 void NonlinearAttitudeControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                       const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
   // Geometric attitude controller
   // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
   // attitude control: Technical report. ETH Zurich, 2013.

   const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
   const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
   const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
   desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
   desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
   desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
   const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
   const Eigen::Vector3d zb = rotmat.col(2);
   desired_thrust_(0) = 0.0;
   desired_thrust_(1) = 0.0;
   desired_thrust_(2) = ref_acc.dot(zb);

   // Update(Att_, q_des_, a_des, targetJerk_); 
   // change curr_att and red_att to NED, ref_acc is already in NED in applyQuasiSlsCtrl()
   // Convert current and reference attitude from ENU to NED
  //  Eigen::Quaterniond curr_att_quat(curr_att(0), curr_att(1), curr_att(2), curr_att(3));
  //  Eigen::Quaterniond ref_att_quat(ref_att(0), ref_att(1), ref_att(2), ref_att(3));
  //  curr_att_quat = px4_ros_com::frame_transforms::enu_to_ned_orientation(curr_att_quat);
  //  ref_att_quat = px4_ros_com::frame_transforms::enu_to_ned_orientation(ref_att_quat);
  // //  curr_att_quat = px4_ros_com::frame_transforms::ros_to_px4_orientation(curr_att_quat);
  // //  ref_att_quat = px4_ros_com::frame_transforms::ros_to_px4_orientation(ref_att_quat); // not working
  //  // Convert back to Eigen::Vector4d
  //  Eigen::Vector4d curr_att_ned = Eigen::Vector4d(curr_att_quat.w(), curr_att_quat.x(), curr_att_quat.y(), curr_att_quat.z());
  //  Eigen::Vector4d ref_att_ned = Eigen::Vector4d(ref_att_quat.w(), ref_att_quat.x(), ref_att_quat.y(), ref_att_quat.z());

  // //  RCLCPP_INFO(rclcpp::get_logger("test"), "curr_att: %f, %f, %f, %f", curr_att(0), curr_att(1), curr_att(2), curr_att(3));
  // //  RCLCPP_INFO(rclcpp::get_logger("test"), "curr_att_ned: %f, %f, %f, %f", curr_att_ned(0), curr_att_ned(1), curr_att_ned(2), curr_att_ned(3));
                                    
  //  const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  //  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att_ned;
  //  const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att_ned);
  //  desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  //  desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  //  desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  //  const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att_ned);
  //  const Eigen::Vector3d zb = rotmat.col(2);
  //  desired_thrust_(0) = 0.0;
  //  desired_thrust_(1) = 0.0;
  //  desired_thrust_(2) = ref_acc.dot(zb);
 }
