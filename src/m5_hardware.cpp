#include "m5_hardware/m5_hardware.hpp"
namespace m5_hardware
{
hardware_interface::CallbackReturn M5Hardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    // ハードウェアインターフェースが初期化されるときに１回だけ呼び出される
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
    joint_names_.clear();
    joint_position_.clear();
    joint_velocities_.clear();
    joint_effort_.clear();
    // ハードウェア情報からジョイント名を取得
    for (const auto & joint : info.joints) {
        joint_names_.push_back(joint.name);
    }
    size_t num_joints = joint_names_.size();
    // どのモーターを使うか関係なくeffort, position, velocity全て用意しておく。
    joint_position_.assign(num_joints, 0);
    joint_velocities_.assign(num_joints, 0);
    joint_effort_.assign(num_joints, 0);
    joint_position_command_.assign(num_joints, 0);
    joint_velocities_command_.assign(num_joints, 0);
    joint_effort_command_.assign(num_joints, 0);

    return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn M5Hardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    // Configure the hardware interface
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> M5Hardware::export_state_interfaces()
{
    // TODO ros2 control用のstate_interfaceを露出させる
    // Export state interfaces
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // 位置と速度のインターフェースを追加
        state_interfaces.emplace_back(joint_names_[i], "position", &joint_position_[i]);
        state_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocities_[i]);
        state_interfaces.emplace_back(joint_names_[i], "effort", &joint_effort_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> M5Hardware::export_command_interfaces()
{
    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // 位置と速度のインターフェースを追加
        command_interfaces.emplace_back(joint_names_[i], "position", &joint_position_command_[i]); // joint_position_command_にros2 controlからの値が入る
        command_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocities_command_[i]);
        command_interfaces.emplace_back(joint_names_[i], "effort", &joint_effort_command_[i]);
    }

    return command_interfaces;
}

hardware_interface::return_type M5Hardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // TODO : M5Stackから各ジョイントの位置と速度を取得
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type M5Hardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // TODO : M5Stackに各ジョイントの位置と速度コマンドを送信
    return hardware_interface::return_type::OK;
}
}  // namespace m5_hardware       

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  m5_hardware::M5Hardware, hardware_interface::SystemInterface)
