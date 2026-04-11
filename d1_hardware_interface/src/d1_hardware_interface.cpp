#include "d1_hardware_interface/d1_hardware_interface.hpp"

#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("D1HardwareInterface");

namespace d1_hardware_interface
{
double D1HardwareInterface::mm_to_deg(const double joint_mm)
{
    double m = 90.0 / 0.033;
    double joint_deg = m * (joint_mm + 0.009) - 30.0;
    return joint_deg;
}
double D1HardwareInterface::deg_to_mm(const double joint_deg)
{
    double m = 0.033 / 90.0;
    double joint_mm = m * (joint_deg + 30.0) - 0.009;
    return joint_mm;
}

std::string D1HardwareInterface::get_param_value(const std::string & param_name, const std::string & default_value)
{
    std::string value = info_.hardware_parameters[param_name];
    if (value.empty())
    {
        return default_value;
    }

    return value;
}

void D1HardwareInterface::to_base()
{
    ServoAngleData servos_angle_commands_;

    std::vector<double> init_commands_;

    if (with_gripper) {
        init_commands_ = {-88.0, -88.0, 88.0, -0.0, 0.0, -0.0, 11.0};                  //////////// ALTEREI VER COM O NILTON
    }
    else{
        init_commands_ = {-88.0, -88.0, 88.0, -0.0, 0.0, -0.0};
    }


    for (uint i = 0; i < n_joints_; ++i) {
        joint_commands_[i].position = (init_commands_[i] * M_PI) / 180.0; // rads
        servos_angle_commands_.angles()[i] = init_commands_[i]; // degrees
    }

    servo_angle_publisher_->Write(servos_angle_commands_, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return;
}

CallbackReturn D1HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(LOGGER, "Configuring Unitree D1 Hardware Interface");

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to init D1 Hardware Interface");
        return CallbackReturn::ERROR;
    }

    info_ = info;
    joints_ = info_.joints;
    n_joints_ = joints_.size();
    std::string gripper = get_param_value("gripper", "");

    std::cout << "Gripper: " << gripper << std::endl;
    std::cout << "N joints: " << n_joints_ << std::endl;

    if (gripper != "") {
        with_gripper = true;
    }

    // std::cout << "TYpe: " << typeid(joints_).name() << std::endl;

    std::string network_interface = get_param_value("network_interface", "");
    std::string topic_sufix = get_param_value("topic_sufix", "");

    if (!network_interface.empty())
        RCLCPP_INFO(LOGGER, "Network Interface: %s", network_interface.c_str());

    joint_states_.resize(n_joints_);
    joint_commands_.resize(n_joints_);
    state_buffer_.resize(n_joints_);
    cmd_buffer_.resize(n_joints_);

    ChannelFactory::Instance()->Init(0, network_interface);

    std::string get_angle_full_topic_ = get_angle_topic_ + topic_sufix;
    servo_angle_subscriber_.reset(new ChannelSubscriber<ServoAngleData>(get_angle_full_topic_));
    servo_angle_subscriber_->InitChannel([this](const void* msg) {
        auto s = static_cast<const ServoAngleData*>(msg);
        std::memcpy(&servos_angle_data_, s, sizeof(ServoAngleData));
    });

    std::string set_angle_full_topic_ = set_angle_topic_ + topic_sufix;
    servo_angle_publisher_.reset(new ChannelPublisher<ServoAngleData>(set_angle_full_topic_));
    servo_angle_publisher_->InitChannel();

    std::string set_power_full_topic_ = set_power_topic_ + topic_sufix;
    servo_power_publisher_.reset(new ChannelPublisher<ServoPower>(set_power_full_topic_));
    servo_power_publisher_->InitChannel();

    to_base();

    // last_write_time_ = std::chrono::steady_clock::now();

    return CallbackReturn::SUCCESS;
}

CallbackReturn D1HardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void) previous_state;
    thread_ = std::thread([this]{ this->io_loop(); });
    return CallbackReturn::SUCCESS;
}

CallbackReturn D1HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void) previous_state;

    if (thread_.joinable()) {
        thread_.join();
    }

    to_base();

    ServoPower servos_power_command_;
    for (uint i = 0; i < n_joints_; ++i) {
        servos_power_command_.power()[i] = 200; // mW
    }

    RCLCPP_INFO(LOGGER, "Deactivating servos.");

    servo_power_publisher_->Write(servos_power_command_);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> D1HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interface;

    for (uint i = 0; i < n_joints_; i++)
    {
        state_interface.emplace_back(
            hardware_interface::StateInterface(
                joints_[i].name, hardware_interface::HW_IF_POSITION, &joint_states_[i].position
            )
        );

        state_interface.emplace_back(
            hardware_interface::StateInterface(
                joints_[i].name, hardware_interface::HW_IF_VELOCITY, &joint_states_[i].velocity
            )
        );
    }

    return state_interface;
}

std::vector<hardware_interface::CommandInterface> D1HardwareInterface::export_command_interfaces()
{

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < n_joints_; ++i) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joints_[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i].position
            )
        );
    }

    return command_interfaces;
}

return_type D1HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;
    ServoAngleData msg;

    // std::memcpy(&msg, &servos_angle_data_, sizeof(msg));

    // for (uint i = 0; i < n_joints_; ++i) {
    //     if (i < 6) {
    //       joint_states_[i].position = msg.angles()[i] * DEG_TO_RAD; // rad
    //     } else {
    //       joint_states_[i].position = this->deg_to_mm(msg.angles()[i]); // rad
    //     }
    //     joint_states_[i].velocity = 0.0;
    // }

    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_states_ = state_buffer_;

    return return_type::OK;
}

return_type D1HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;

    // ServoAngleData servos_angle_commands_;

    // for (uint i = 0; i < n_joints_; ++i) {
    //     if (i < 6) {
    //       servos_angle_commands_.angles()[i] = joint_commands_[i].position * RAD_TO_DEG; // degrees
    //     } else {
    //       servos_angle_commands_.angles()[i] = this->mm_to_deg(joint_commands_[i].position); // degrees
    //     }
    // }

    // servo_angle_publisher_->Write(servos_angle_commands_, 0);

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_buffer_ = joint_commands_;

    return return_type::OK;
}

void D1HardwareInterface::io_loop()
{
    // std::vector<double> last_position(n_joints_, 0.0);
    ServoAngleData cmd_msg;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (uint i = 0; i < n_joints_; ++i) {
            cmd_msg.angles()[i] = (i < 6)
                    ? cmd_buffer_[i].position * RAD_TO_DEG
                    : mm_to_deg(cmd_buffer_[i].position);
        }
    }

    servo_angle_publisher_->Write(cmd_msg, 0);

    ServoAngleData state_msg;
    std::memcpy(&state_msg, &servos_angle_data_, sizeof(state_msg));
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (uint i = 0; i < n_joints_; ++i) {
            state_buffer_[i].position = state_msg.angles()[i];
            state_buffer_[i].velocity = 0.0;
        }
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(d1_hardware_interface::D1HardwareInterface, hardware_interface::SystemInterface);