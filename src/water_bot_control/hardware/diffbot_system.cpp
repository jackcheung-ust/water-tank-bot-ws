// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "water_bot_control/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace water_bot_control
{

hardware_interface::CallbackReturn WarterBotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  RCLCPP_INFO(get_logger(),"wheel _radius : %f",wheel_radius_);
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  //the current system has extact two gpio interface
  if(info_.gpios.size()!=1){
    RCLCPP_FATAL(
      get_logger(), "waterbot hardware system has '%ld' GPIO components, but '%d' expected.",
      info_.gpios.size(), 1);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if(info_.gpios[0].command_interfaces.size() != 1){
    RCLCPP_FATAL(
        get_logger(), "GPIO component %s has '%ld' command interfaces, '%d' expected.",
        info_.gpios[0].name.c_str(), info_.gpios[0].command_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
  }

  if(info_.gpios[0].state_interfaces.size() != 2){
    RCLCPP_FATAL(
      get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
      info_.gpios[0].name.c_str(), info_.gpios[0].state_interfaces.size(), 2);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WarterBotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  //Connect the Serial with the hardware 
  serialcom_.connect("/dev/ttyUSB0",1000000,50);

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  for (const auto & [name, descr] : gpio_command_interfaces_){
    set_command(name, 0.0);
  }

  for (const auto & [name, descr] : gpio_state_interfaces_){
    set_state(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WarterBotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  //Check if the Serial com open correctly 

  if (!serialcom_.is_Opened()){

    RCLCPP_FATAL(get_logger(),"Serial Com is not opened ! exiting ...");
    
    return hardware_interface::CallbackReturn::ERROR;
  }

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

   for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WarterBotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  serialcom_.disconnect();
 

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WarterBotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);
  auto [left_feedback_v, right_feedback_v] = serialcom_.read_wheel_vel();
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      double actual_vel = 0.0; 
      if (descr.get_prefix_name()=="left_wheel_joint"){
        actual_vel = double(left_feedback_v/wheel_radius_);
      }
      else if (descr.get_prefix_name()=="right_wheel_joint"){
        actual_vel = double(right_feedback_v/wheel_radius_);
      }

      set_state(name, actual_vel);
      
      ss << std::endl << "\t" << "velocity in read function : " << actual_vel << " for '" << name << "'!";
    }
  }
  
 set_state(
    info_.gpios[0].name+ "/" + info_.gpios[0].state_interfaces[0].name,
    static_cast<double>(serialcom_.get_op_mode())
    );

  set_state(
    info_.gpios[0].name+ "/" + info_.gpios[0].state_interfaces[1].name,
    static_cast<double>(serialcom_.get_clean_mode())
    );

  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type water_bot_control ::WarterBotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  std::stringstream ss;
  ss << "Writing commands:";
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    if ( descr.get_prefix_name() == "left_wheel_joint"){
      left_v_rad_ = get_command(name);
    }
    else if (descr.get_prefix_name() == "right_wheel_joint"){
      right_v_rad_ = get_command(name);
    }

    // ss << std::fixed << std::setprecision(2) << std::endl
    //    << "\t" << "command " << get_command(name) << " for '" << name << "'!";
    
  }

  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    if (descr.get_prefix_name() == "water_bot_IOs"){
      clean_mode_command = get_command(name);
      ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_command(name) << " for GPIO output '" << name << "'";
    }
  }

  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // double left_command = left_v_rad_ * wheel_radius_;
  // double right_command = right_v_rad_ * wheel_radius_;
  //convert rad/s into linear velocity 
  int16_t left_command = static_cast<int16_t>(left_v_rad_ * wheel_radius_);
  int16_t right_command = static_cast<int16_t>(right_v_rad_ * wheel_radius_);
  std::stringstream ss1;
  // ss1 << std::endl << "\t" <<"converted left command :" << left_v_command << "...";
  // ss1 << std::endl << "\t" <<"converted right command :" << right_v_command << "...";
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss1.str().c_str());
  //write command v to the hardware 
  int16_t clean_mode_command_sent = 0;
  if (clean_mode_command >= 0.0 && clean_mode_command <= 2.0) {
    clean_mode_command_sent = static_cast<int16_t>(std::round(clean_mode_command));
  } else {
    RCLCPP_WARN(get_logger(), "Clean mode command out of bounds: %f. Using 0.", clean_mode_command);
    clean_mode_command_sent = 0;
  }

  // ss1 << "clean mode from the command interface : " << clean_mode_command << std::endl;
  // ss1 << "coverted clean mode command : " << clean_mode_command_sent << std::endl;
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss1.str().c_str());
  serialcom_.write_data(left_command,right_command,clean_mode_command_sent);

  return hardware_interface::return_type::OK;
}

}  // namespace water_bot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  water_bot_control::WarterBotHardware, hardware_interface::SystemInterface)
