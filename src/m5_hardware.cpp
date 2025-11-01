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
    // パラメータ（URDF/ros2_controlのhardware_parameters）から取得する想定
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;
    if (info_.hardware_parameters.count("port")) { port = info_.hardware_parameters.at("port"); }
    if (info_.hardware_parameters.count("baud")) { baud = std::stoi(info_.hardware_parameters.at("baud")); }

    // モード配列を準備（既定は POSITION）
    cmd_mode_.assign(joint_names_.size(), CmdMode::POSITION);

    // コマンドインターフェースから送信モードを決定
    for (size_t i=0;i<info_.joints.size();++i) {
        bool has_pos=false, has_eff=false, has_vel=false;
        for (const auto &ci : info_.joints[i].command_interfaces) {
        if (ci.name == "position") has_pos=true;
        if (ci.name == "effort")   has_eff=true;
        if (ci.name == "velocity") has_vel=true;
        }
        if (!info_.hardware_parameters.count("cmd_mode")) {
        if (has_eff) cmd_mode_[i] = CmdMode::EFFORT;
        else if (has_vel) cmd_mode_[i] = CmdMode::VELOCITY;
        else                     cmd_mode_[i] = CmdMode::POSITION;
        }
    }

    // 状態保持用配列の準備(state interfaceが指定されているかどうか)
    const size_t N = info_.joints.size();
    has_state_pos_.assign(N, false);
    has_state_vel_.assign(N, false);
    has_state_eff_.assign(N, false);

    for (size_t i = 0; i < N; ++i) {
        for (const auto &si : info_.joints[i].state_interfaces) {
            if (si.name == "position") has_state_pos_[i] = true;
            else if (si.name == "velocity") has_state_vel_[i] = true;
            else if (si.name == "effort")   has_state_eff_[i] = true;
        }
    }
    // シリアル開始
    m5_ = std::make_unique<m5link::M5SerialClient>();
    if (!m5_->start(port, baud, joint_names_.size())) {
        RCLCPP_ERROR(rclcpp::get_logger("M5Hardware"), "Failed to open %s", port.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
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
    if (!m5_) {
        return hardware_interface::return_type::OK;
    }

    m5link::JointStateSnapshot snap;
    if (!m5_->tryGetLatestState(snap)) {
        // 新しい状態がまだ無い／更新なし
        return hardware_interface::return_type::OK;
    }

    const size_t N = joint_names_.size();

    // position
    if (!snap.position.empty()) {
        const size_t n = std::min(N, snap.position.size());
        for (size_t i = 0; i < n; ++i) {
        if (i < has_state_pos_.size() && !has_state_pos_[i]) continue;
        joint_position_[i] = snap.position[i];
        }
    }

    // velocity
    if (!snap.velocity.empty()) {
        const size_t n = std::min(N, snap.velocity.size());
        for (size_t i = 0; i < n; ++i) {
        if (i < has_state_vel_.size() && !has_state_vel_[i]) continue;
        joint_velocities_[i] = snap.velocity[i];
        }
    }

    // effort
    if (!snap.effort.empty()) {
        const size_t n = std::min(N, snap.effort.size());
        for (size_t i = 0; i < n; ++i) {
        if (i < has_state_eff_.size() && !has_state_eff_[i]) continue;
        joint_effort_[i] = snap.effort[i];
        }
    }

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type M5Hardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // N本の配列を用意（同じ長さ・同じ順序）
    const size_t N = joint_names_.size();
    std::vector<double> pos(N, 0.0), eff(N, 0.0), vel(N, 0.0);

    for (size_t i=0;i<N;++i) {
        if (cmd_mode_[i] == CmdMode::POSITION) {
            pos[i] = joint_position_command_[i]; // position 指令を使用
            vel[i] = 0.0;
            eff[i] = 0.0;                         // 未使用側は0.0（M5で無視）
        }
        else if (cmd_mode_[i] == CmdMode::EFFORT) {
            pos[i] = 0.0;
            vel[i] = 0.0;
            eff[i] = joint_effort_command_[i];   // effort 指令を使用
        }
        else if (cmd_mode_[i] == CmdMode::VELOCITY) {
            pos[i] = 0.0;
            vel[i] = joint_velocities_command_[i];
            eff[i] = 0.0;                         // 未使用側は0.0（M5で無視）
        }
    }

    if (m5_) m5_->sendSetCommand(pos, eff);
    return hardware_interface::return_type::OK;
}
}  // namespace m5_hardware       

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  m5_hardware::M5Hardware, hardware_interface::SystemInterface)
