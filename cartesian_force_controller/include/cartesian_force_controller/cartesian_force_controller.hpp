////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_force_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_FORCE_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_FORCE_CONTROLLER_HPP_INCLUDED

// Project
#include "cartesian_controller_base/Utility.h"
#include <cartesian_force_controller/cartesian_force_controller.h>

// Other
#include <algorithm>

namespace cartesian_force_controller
{

template <class HardwareInterface>
CartesianForceController<HardwareInterface>::
CartesianForceController()
: Base::CartesianControllerBase()
, m_hand_frame_control(true)
, m_lp_filter_initialized(false)
, m_notch_filter_initialized(false)
, m_tool_speed_initialized(false)
{
}

template <class HardwareInterface>
bool CartesianForceController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  Base::init(hw,nh);

  if (!nh.getParam("ft_sensor_ref_link",m_ft_sensor_ref_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/ft_sensor_ref_link" << " from parameter server");
    return false;
  }

  // Make sure sensor link is part of the robot chain
  if(!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    ROS_ERROR_STREAM(m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                           << Base::m_robot_base_link << " to "
                                           << Base::m_end_effector_link);
    return false;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_signal_taring_server = nh.advertiseService("signal_taring",&CartesianForceController<HardwareInterface>::signalTaringCallback,this);
  m_target_wrench_subscriber = nh.subscribe("target_wrench",2,&CartesianForceController<HardwareInterface>::targetWrenchCallback,this);
  m_ft_sensor_wrench_subscriber = nh.subscribe("ft_sensor_wrench",2,&CartesianForceController<HardwareInterface>::ftSensorWrenchCallback,this);
  m_tool_speed_subscriber = nh.subscribe("mirka_driver/average_speed", 2,&CartesianForceController<HardwareInterface>::toolSpeedCallback, this);

  m_ft_sensor_wrench_publisher =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> >(nh, "wrench", 3);
  m_ft_sensor_wrench_filt_publisher =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> >(nh, "wrench_filtered", 3);

  // Initialize Filters: sampling frequency and cut-off frequency
  if (!nh.getParam("filter_fs",m_fs))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/filter_fs" << " from parameter server");
    return false;
  }
  if (!nh.getParam("filter_lp_fc",m_fc))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/filter_lp_fc" << " from parameter server");
    return false;
  }
  if (!nh.getParam("filter_n_bw",m_bw))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/filter_n_bw" << " from parameter server");
    return false;
  }

  // Initialize tool and gravity compensation
  std::map<std::string, double> gravity;
  if (!nh.getParam("gravity",gravity))
  {
    ROS_INFO_STREAM("Failed to load " << nh.getNamespace() + "/gravity" << " from parameter server");
    gravity["x"] = 0;
    gravity["y"] = 0;
    gravity["z"] = 0;
  }
  std::map<std::string, double> tool;
  if (!nh.getParam("tool",tool))
  {
    ROS_INFO_STREAM("Failed to load " << nh.getNamespace() + "/tool" << " from parameter server");
    tool["com_x"] = 0;
    tool["com_y"] = 0;
    tool["com_z"] = 0;
  }
  // In sensor frame
  m_center_of_mass = ctrl::Vector3D(tool["com_x"],tool["com_y"],tool["com_z"]);

  // In base frame
  m_weight_force.head<3>() = tool["mass"] * ctrl::Vector3D(gravity["x"],gravity["y"],gravity["z"]);
  m_weight_force.tail<3>() = ctrl::Vector3D::Zero();  // Update in control cycle
  m_grav_comp_during_taring = -m_weight_force;

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type = std::bind(
      &CartesianForceController::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  m_dyn_conf_server.reset(
      new dynamic_reconfigure::Server<Config>(nh));

  m_dyn_conf_server->setCallback(m_callback_type);

  return true;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
starting(const ros::Time& time)
{
  Base::starting(time);
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
stopping(const ros::Time& time)
{
}

template <>
void CartesianForceController<hardware_interface::VelocityJointInterface>::
stopping(const ros::Time& time)
{
    // Stop drifting by sending zero joint velocities
    Base::computeJointControlCmds(ctrl::Vector6D::Zero(), ros::Duration(0));
    Base::writeJointControlCmds();
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  ros::Duration internal_period(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error,internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
computeForceError()
{
  ctrl::Vector6D target_wrench;
  if (m_hand_frame_control) // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link);
  }
  else // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench,m_new_ft_sensor_ref)
    + target_wrench
    + compensateGravity();
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
setFtSensorReferenceFrame(const std::string& new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      sensor_ref,
      m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      new_sensor_ref,
      m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
compensateGravity()
{
  ctrl::Vector6D compensating_force = ctrl::Vector6D::Zero();

  // Compute actual gravity effects in sensor frame
  ctrl::Vector6D tmp = Base::displayInTipLink(m_weight_force,m_ft_sensor_ref_link);
  tmp.tail<3>() = m_center_of_mass.cross(tmp.head<3>()); // M = r x F

  // Display in base link
  m_weight_force = Base::displayInBaseLink(tmp,m_ft_sensor_ref_link);

  // Add actual gravity compensation
  compensating_force -= m_weight_force;

  // Remove deprecated terms from moment of taring
  compensating_force -= m_grav_comp_during_taring;

  return compensating_force;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  m_target_wrench[0] = wrench.wrench.force.x;
  m_target_wrench[1] = wrench.wrench.force.y;
  m_target_wrench[2] = wrench.wrench.force.z;
  m_target_wrench[3] = wrench.wrench.torque.x;
  m_target_wrench[4] = wrench.wrench.torque.y;
  m_target_wrench[5] = wrench.wrench.torque.z;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
toolSpeedCallback(const std_msgs::UInt16& speed)
{
    if (speed.data < 3000)
  {
    m_tool_speed_initialized = false;
    return;
  }

  m_tool_speed_rpm = speed.data;
  m_tool_speed_initialized = true;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench.wrench.force.x;
  tmp[1] = wrench.wrench.force.y;
  tmp[2] = wrench.wrench.force.z;
  tmp[3] = wrench.wrench.torque.x;
  tmp[4] = wrench.wrench.torque.y;
  tmp[5] = wrench.wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;

  if(m_tool_speed_initialized)
  {
    ctrl::Vector6D tmp_vec;
    tmp_vec.head<3>() = Eigen::Vector3d(tmp.force.x(), tmp.force.y(), tmp.force.z());
    tmp_vec.tail<3>() = Eigen::Vector3d(tmp.torque.x(), tmp.torque.y(), tmp.torque.z());
    double tool_speed_hz = static_cast<double>(m_tool_speed_rpm) / 60.0 + m_tool_interaction_hz;

    applyNotchFilter(tool_speed_hz, tmp_vec, m_ft_sensor_notch_filt_wrench);
  }
  else
  {
    for(int i = 0; i < 6; i++)
    {
      m_ft_sensor_notch_filt_wrench[i] = tmp[i];
    }
  }

  applyLPFilter(m_ft_sensor_notch_filt_wrench, m_ft_sensor_lp_filt_wrench);

  for(int i = 0; i < 6; i++)
  {
    m_ft_sensor_wrench[i] = m_ft_sensor_lp_filt_wrench[i];
  }

  // Publish
  auto now = ros::Time::now();
  if (m_ft_sensor_wrench_publisher->trylock())
  {
    m_ft_sensor_wrench_publisher->msg_.header.stamp = now;
    m_ft_sensor_wrench_publisher->msg_.header.frame_id = Base::m_end_effector_link;
    m_ft_sensor_wrench_publisher->msg_.wrench.force.x = tmp[0];
    m_ft_sensor_wrench_publisher->msg_.wrench.force.y = tmp[1];
    m_ft_sensor_wrench_publisher->msg_.wrench.force.z = tmp[2];
    m_ft_sensor_wrench_publisher->msg_.wrench.torque.x = tmp[3];
    m_ft_sensor_wrench_publisher->msg_.wrench.torque.y = tmp[4];
    m_ft_sensor_wrench_publisher->msg_.wrench.torque.z = tmp[5];

    m_ft_sensor_wrench_publisher->unlockAndPublish();
  }
  if (m_ft_sensor_wrench_filt_publisher->trylock())
  {
    m_ft_sensor_wrench_filt_publisher->msg_.header.stamp = now;
    m_ft_sensor_wrench_filt_publisher->msg_.header.frame_id = Base::m_end_effector_link;
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.force.x = m_ft_sensor_wrench[0];
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.force.y = m_ft_sensor_wrench[1];
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.force.z = m_ft_sensor_wrench[2];
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.torque.x = m_ft_sensor_wrench[3];
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.torque.y = m_ft_sensor_wrench[4];
    m_ft_sensor_wrench_filt_publisher->msg_.wrench.torque.z = m_ft_sensor_wrench[5];

    m_ft_sensor_wrench_filt_publisher->unlockAndPublish();
  }
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
applyLPFilter(const ctrl::Vector6D& measured_wrench, ctrl::Vector6D& filtered_wrench)
{
  double m_alpha = (2 * M_PI * m_fc)/(2 * M_PI * m_fc + m_fs);

  if (!m_lp_filter_initialized)
  {
    for(int i = 0; i < 6; i++)
    {
      filtered_wrench[i] = measured_wrench[i];
    }
    m_lp_filter_initialized = true;
  }
  else
  {
    // Apply LP filter: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    for(int i = 0; i < 6; i++)
    {
      filtered_wrench[i] = m_alpha * measured_wrench[i] + (1 - m_alpha) * filtered_wrench[i];
    }
  }
  return;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
applyNotchFilter(const double& f0, const ctrl::Vector6D& measured_wrench, ctrl::Vector6D& filtered_wrench)
{
  double Q = f0 / m_bw;
  double w0 = 2.0* M_PI * f0 / m_fs;
  double alpha_notch = sin(w0) / (2.0 * Q);

  double a0 = 1.0 + alpha_notch;
  double a1 = -2.0 * cos(w0) / a0;
  double a2 = (1.0 - alpha_notch) / a0;
  double b0 = 1.0 / a0;
  double b1 = -2.0 * cos(w0) / a0;
  double b2 = 1.0 / a0;

  static double m_notch_x[6][3] = {{0.0}};
  static double m_notch_y[6][3] = {{0.0}};
  
  if(!m_notch_filter_initialized)
  {
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        m_notch_x[i][j] = measured_wrench[i];
        m_notch_y[i][j] = measured_wrench[i];
      }
    }
    m_notch_filter_initialized = true;
  }
  else
  {
    for(int i = 0; i < 6; i++)
    {
      m_notch_x[i][2] = m_notch_x[i][1];
      m_notch_x[i][1] = m_notch_x[i][0];
      m_notch_x[i][0] = measured_wrench[i];

      m_notch_y[i][2] = m_notch_y[i][1];
      m_notch_y[i][1] = m_notch_y[i][0];
      // Apply Notch filter: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
      m_notch_y[i][0] = b0 * m_notch_x[i][0] + b1 * m_notch_x[i][1] + b2 * m_notch_x[i][2] - a1 * m_notch_y[i][1] - a2 * m_notch_y[i][2];

      filtered_wrench[i] = m_notch_y[i][0];
    }
  }
  return;
}

template <class HardwareInterface>
bool CartesianForceController<HardwareInterface>::
signalTaringCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // Compute current gravity effects in sensor frame
  ctrl::Vector6D tmp = Base::displayInTipLink(m_weight_force,m_ft_sensor_ref_link);
  tmp.tail<3>() = m_center_of_mass.cross(tmp.head<3>()); // M = r x F

  // Taring the sensor is like adding a virtual force that exactly compensates
  // the weight force.
  tmp = -tmp;

  // Display in base link
  m_grav_comp_during_taring = Base::displayInBaseLink(tmp,m_ft_sensor_ref_link);

  res.message = "Got it.";
  res.success = true;
  return true;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::dynamicReconfigureCallback(Config& config,
                                                                             uint32_t level)
{
  m_hand_frame_control = config.hand_frame_control;
}

}

#endif
