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
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_force_controller/cartesian_force_controller.h>

#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_force_controller
{
CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(true), m_lp_filter_initialized(false), m_notch_filter_initialized(false), m_tool_speed_initialized(false)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_ft_sensor_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_ft_sensor_wrench_publisher =
  std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
    get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      std::string(get_node()->get_name()) + "/wrench", 3));
  m_ft_sensor_wrench_filt_publisher =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
        get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
          std::string(get_node()->get_name()) + "/wrench_filtered", 3));

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));
  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      std::string("/ft_sensor_wrench"), 10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));
  m_tool_speed_subscriber =
    get_node()->create_subscription<std_msgs::msg::UInt16>(
      std::string("/mirka_driver/average_speed"), 10,
      std::bind(&CartesianForceController::toolSpeedCallback, this, std::placeholders::_1));

  // Initialize Filters: sampling frequency, cut-off frequency, bandwidth
  if(!get_node()->has_parameter("filter_fs"))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "filter_fs parameter is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_fs = get_node()->get_parameter("filter_fs").as_double();

  if(!get_node()->has_parameter("filter_lp_fc"))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "filter_lp_fc parameter is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_fc = get_node()->get_parameter("filter_lp_fc").as_double();

  if(!get_node()->has_parameter("filter_n_bw"))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "filter_n_bw parameter is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_bw = get_node()->get_parameter("filter_n_bw").as_double();

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianForceController::update(const rclcpp::Time & time,
                                                                   const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error, internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control)  // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench, Base::m_end_effector_link);
  }
  else  // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench, m_new_ft_sensor_ref) + target_wrench;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string & new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

void CartesianForceController::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }

  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianForceController::toolSpeedCallback(const std_msgs::msg::UInt16::SharedPtr speed)
{
  if (!this->isActive() || speed->data < 3000)
  {
    m_tool_speed_initialized = false;
    return;
  }

  m_tool_speed_rpm = speed->data;
  m_tool_speed_initialized = true;

  return;
}

void CartesianForceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;
  tmp[3] = wrench->wrench.torque.x;
  tmp[4] = wrench->wrench.torque.y;
  tmp[5] = wrench->wrench.torque.z;

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
  auto now = rclcpp::Clock().now();
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

void CartesianForceController::applyLPFilter(const ctrl::Vector6D& measured_wrench, ctrl::Vector6D& filtered_wrench)
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

void CartesianForceController::applyNotchFilter(const double& f0, const ctrl::Vector6D& measured_wrench, ctrl::Vector6D& filtered_wrench)
{
  double Q = f0 / m_bw;
  double w0 = 2.0* M_PI * f0 / m_fs;
  double m_alpha_notch = sin(w0) / (2.0 * Q);

  double a0 = 1.0 + m_alpha_notch;
  double a1 = -2.0 * cos(w0) / a0;
  double a2 = (1.0 - m_alpha_notch) / a0;
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


}  // namespace cartesian_force_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController,
                       controller_interface::ControllerInterface)
