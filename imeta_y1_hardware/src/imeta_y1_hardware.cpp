#include "imeta_y1_hardware/imeta_y1_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>
#include <glog/logging.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace imeta_y1_hardware {

IMetaY1HW::IMetaY1HW() = default;

hardware_interface::CallbackReturn IMetaY1HW::on_init(
    const hardware_interface::HardwareInfo& info) {

  try {
    std::string can_interface = info.hardware_parameters.at("can_interface");
    int arm_end_type = std::stoi(info.hardware_parameters.at("arm_end_type")); //0: only arm, 1: gripper_T, 2: gripper_G, 3: gripper_GT
    bool enable_arm = info.hardware_parameters.at("enable_arm") == "true";

    // get urdf path
    std::string package_path =
        ament_index_cpp::get_package_share_directory("imeta_y1_description");
    std::string urdf_path;
    if (arm_end_type == 0) {
      // only load robotic arm
      urdf_path = package_path + "/urdf/imeta_y1.urdf";
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("IMetaY1HW"), "arm_end_type is %d , not supported",
                  arm_end_type);
      return CallbackReturn::ERROR;
    }

    y1_sdk_interface_ = std::make_unique<imeta::y1_controller::Y1SDKInterface>(
        can_interface, urdf_path, arm_end_type, enable_arm);

    y1_sdk_interface_->Init();

    y1_sdk_interface_->SetArmControlMode(imeta::y1_controller::Y1SDKInterface::ControlMode::NRT_JOINT_POSITION);

    for (const auto& joint_info : info.joints) {
      std::string joint_name = joint_info.name;
      joint_names_.push_back(joint_name);
    }

    // Initialize state and command vectors with proper size
    size_t num_joints = joint_names_.size();
    pos_states_.resize(num_joints, 0.0);
    vel_states_.resize(num_joints, 0.0);
    tau_states_.resize(num_joints, 0.0);
    pos_commands_.resize(num_joints, 0.0);
    // vel_commands_.resize(num_joints, 0.0);
    // tau_commands_.resize(num_joints, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("IMetaY1HW"), 
                "Initialized with CAN: %s, URDF: %s, arm_end_type: %d, enable_arm: %s, joints: %zu",
                can_interface.c_str(), urdf_path.c_str(), arm_end_type, 
                enable_arm ? "true" : "false", num_joints);
  } catch (const std::out_of_range& e) {
    RCLCPP_ERROR(rclcpp::get_logger("IMetaY1HW"), 
                 "Missing required hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("IMetaY1HW"), 
                 "Error initializing Y1 SDK: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMetaY1HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IMetaY1HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IMetaY1HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //     joint_names_[i], hardware_interface::HW_IF_VELOCITY,
    //     &vel_commands_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //     joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn IMetaY1HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Read initial joint states from hardware
  try {
    auto pos_states = y1_sdk_interface_->GetJointPosition();
    
    // Initialize commands to current position to avoid sudden movements
    for (size_t i = 0; i < pos_states.size(); ++i) {
      pos_commands_[i] = pos_states[i];
    }
    
    RCLCPP_INFO(rclcpp::get_logger("IMetaY1HW"), 
                "IMETA Y1 activated with initial joint positions");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("IMetaY1HW"), 
                 "Failed to read initial joint states: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMetaY1HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("IMetaY1HW"), "IMETA Y1 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMetaY1HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // check error
  std::vector<int> error_code = y1_sdk_interface_->GetJointErrorCode();
  for (size_t i = 0; i < error_code.size(); ++i) {
    if (error_code[i] != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("IMetaY1HW"),
                   "Joint %zu error code: %d", i, error_code[i]);
    }
  }
  // get state from Y1 SDK
  auto pos_states = y1_sdk_interface_->GetJointPosition();
  auto vel_states = y1_sdk_interface_->GetJointVelocity();
  auto tau_states = y1_sdk_interface_->GetJointEffort();

  // This copies data without changing vector addresses, keeping StateInterface pointers valid
  std::memcpy(pos_states_.data(), pos_states.data(), pos_states.size() * sizeof(double));
  std::memcpy(vel_states_.data(), vel_states.data(), vel_states.size() * sizeof(double));
  std::memcpy(tau_states_.data(), tau_states.data(), tau_states.size() * sizeof(double));
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMetaY1HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  std::array<double, 6> joint_position;
  for (int i = 0; i < 6; i++) {
    joint_position[i] = pos_commands_[i];
  }
  int velocity_ratio = 10; // Set velocity ratio as needed
  y1_sdk_interface_->SetArmJointPosition(joint_position, velocity_ratio);
  return hardware_interface::return_type::OK;
}

}  // namespace imeta_y1_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(imeta_y1_hardware::IMetaY1HW,
                       hardware_interface::SystemInterface)