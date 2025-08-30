#ifndef M5_HARDWARE__M5_HARDWARE_HPP_
#define M5_HARDWARE__M5_HARDWARE_HPP_
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
namespace m5_hardware
{
    class M5Hardware : public hardware_interface::SystemInterface
    {

    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(
            const rclcpp::Time & time,const rclcpp::Duration & period) override;
        hardware_interface::return_type write(
            const rclcpp::Time & time,const rclcpp::Duration & period) override;
    private:
        std::vector<std::string> joint_names_;  // ジョイント名のリスト
        std::vector<double> joint_position_;  // 関節の位置を保持する配列
        std::vector<double> joint_velocities_;  // 関節の速度を保持する配列
        std::vector<double> joint_effort_;  // 関節の電流を保持する配列
        std::vector<double> joint_position_command_;  // 関節の位置コマンドを保持する配列
        std::vector<double> joint_velocities_command_;  // 関節の速度コマンドを保持する配列
        std::vector<double> joint_effort_command_;  // 関節の電流コマンドを保持する配列

    };
}
#endif  // M5_HARDWARE__M5_HARDWARE_HPP_