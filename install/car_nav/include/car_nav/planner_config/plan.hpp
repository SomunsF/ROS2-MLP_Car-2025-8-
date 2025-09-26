// src/end_to_end_controller.hpp

#ifndef END_TO_END_CONTROLLER_HPP_
#define END_TO_END_CONTROLLER_HPP_

#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include <torch/script.h> // LibTorch头文件
#include <torch/torch.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

namespace zzzplan
{

class EndToEndController : public nav2_core::Controller
{
public:
  EndToEndController() = default;
  ~EndToEndController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("EndToEndController")};

  // LibTorch模型
  torch::jit::script::Module module_;

  // ROS订阅者
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // 数据存储
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  sensor_msgs::msg::Imu::SharedPtr current_imu_;
  nav_msgs::msg::Path current_path_;
  
  // 参数
  double lookahead_distance_;
  double min_lookahead_distance_;
  double goal_tolerance_;
  int input_size_;
  int output_size_;
  double max_linear_velocity_;
  double min_linear_velocity_;
  
  // 状态变量
  std::vector<float> last_valid_goal_;
  int goal_reached_count_;
  
  // 角速度平滑相关
  double last_angular_vel_;
  std::vector<double> angular_history_;
  static constexpr size_t ANGULAR_HISTORY_SIZE = 5;
  
  // 回调函数
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  // 辅助函数
  std::vector<float> computeLocalGoal(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & path);
  
  torch::Tensor prepareModelInput(
    const sensor_msgs::msg::LaserScan & scan,
    const sensor_msgs::msg::Imu & imu,
    const std::vector<float> & local_goal);
  
  double calculatePathCurvature(
    const nav_msgs::msg::Path & path,
    size_t start_idx,
    size_t end_idx);
};

} // namespace zzzplan

#endif // END_TO_END_CONTROLLER_HPP_