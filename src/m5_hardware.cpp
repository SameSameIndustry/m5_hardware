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
    // infoからros2 control.xacroで指定したstate interfaceを取得し、対応する変数を登録していく
    std::vector<hardware_interface::StateInterface> sis;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        for (const auto &si : info_.joints[i].state_interfaces) {
        const auto &name = si.name;
            if (name == "position") {
                sis.emplace_back(joint_names_[i], name, &joint_position_[i]);
            } else if (name == "velocity") {
                sis.emplace_back(joint_names_[i], name, &joint_velocities_[i]);
            } else if (name == "effort") {
                sis.emplace_back(joint_names_[i], name, &joint_effort_[i]);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("M5Hardware"),
                            "Unsupported state IF '%s' for joint '%s' (ignored).",
                            name.c_str(), joint_names_[i].c_str());
            }
        }
    }
  return sis;
}

std::vector<hardware_interface::CommandInterface> M5Hardware::export_command_interfaces()
{
    // infoからros2 control.xacroで指定したcommand interfaceを取得し、対応する変数を登録していく
    std::vector<hardware_interface::CommandInterface> cis;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        for (const auto &ci : info_.joints[i].command_interfaces) {
            const auto &name = ci.name;
            if (name == "position") {
                cis.emplace_back(joint_names_[i], name, &joint_position_command_[i]);
            } else if (name == "velocity") {
                cis.emplace_back(joint_names_[i], name, &joint_velocities_command_[i]);
            } else if (name == "effort") {
                cis.emplace_back(joint_names_[i], name, &joint_effort_command_[i]);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("M5Hardware"),
                            "Unsupported command IF '%s' for joint '%s' (ignored).",
                            name.c_str(), joint_names_[i].c_str());
            }
        }
    }
    return cis;
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
