#ifndef UNITREE_D1_HW_INTERFACE_HPP__
#define UNITREE_D1_HW_INTERFACE_HPP__

#include <atomic>
#include <mutex>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "d1_hardware_interface/visibility_control.h"
#include "ServoAngleData.hpp"
#include "ServoPower.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#define GetServoAnglesTopic "servo_angle_data"
#define SetServoAnglesTopic "servo_angle_command"
#define DeactivateServosTopic "deactivate_servos"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using namespace unitree::robot;

#define M_PI           3.14159265358979323846

namespace d1_hardware_interface
{

struct Joint_State
{
    double position;
    double velocity;
};

struct Joint_Command
{
    double position;
};

class D1HardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(D1HardwareInterface)

    D1_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    D1_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    D1_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    D1_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    D1_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    D1_HARDWARE_INTERFACE_PUBLIC
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    D1_HARDWARE_INTERFACE_PUBLIC
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void to_base(); // Move the robot to initial position

    // Create and assign parameters
    std::string get_param_value(const std::string & param_name, const std::string & default_value);

    double mm_to_deg(const double joint_mm);

    double deg_to_mm(const double joint_deg);

    double RAD_TO_DEG = 180.0 / M_PI;
    double DEG_TO_RAD = M_PI / 180.0;

    std::vector<Joint_State> joint_states_;
    std::vector<Joint_Command> joint_commands_;

    // joints info holder
    hardware_interface::HardwareInfo info_;
    std::vector<hardware_interface::ComponentInfo> joints_;
    uint n_joints_;
    bool with_gripper = false;

    // base DDS topics
    const std::string get_angle_topic_ = "servo_angle_data";
    const std::string set_angle_topic_ = "servo_angle_command";
    const std::string set_power_topic_ = "deactivate_servos";

    ServoAngleData servos_angle_data_; // msg
    ChannelSubscriberPtr<ServoAngleData> servo_angle_subscriber_;
    ChannelPublisherPtr<ServoAngleData> servo_angle_publisher_;
    ChannelPublisherPtr<ServoPower> servo_power_publisher_;
};

}  // namespace d1_hardware_interface

#endif  // UNITREE_D1_HW_INTERFACE_HPP__