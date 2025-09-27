// src/end_to_end_controller.cpp
#include "nav2_util/node_utils.hpp"
#include "plan.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/goal_checker.hpp" // 确保包含 goal_checker 头文件

namespace zzzplan
{

void EndToEndController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // 声明参数
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".model_path", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".output_size", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.15));

  // 获取参数
  std::string model_path;
  node->get_parameter(plugin_name_ + ".model_path", model_path);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".output_size", output_size_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".min_linear_velocity", min_linear_velocity_);
  node->get_parameter(plugin_name_ + ".min_lookahead_distance", min_lookahead_distance_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);

  // 初始化状态变量
  last_valid_goal_ = {0.0f, 0.0f};
  goal_reached_count_ = 0;
  last_angular_vel_ = 0.0;
  angular_history_.clear();

  // 打印加载的参数值，确认参数文件生效
  RCLCPP_INFO(logger_, "Loaded parameters from config file:");
  RCLCPP_INFO(logger_, "  - lookahead_distance: %.3f", lookahead_distance_);
  RCLCPP_INFO(logger_, "  - min_lookahead_distance: %.3f", min_lookahead_distance_);
  RCLCPP_INFO(logger_, "  - goal_tolerance: %.3f", goal_tolerance_);
  RCLCPP_INFO(logger_, "  - max_linear_velocity: %.3f", max_linear_velocity_);
  RCLCPP_INFO(logger_, "  - min_linear_velocity: %.3f", min_linear_velocity_);
  RCLCPP_INFO(logger_, "  - output_size: %d", output_size_);
  RCLCPP_INFO(logger_, "  - angular_velocity: UNLIMITED (no restrictions)");

  // 加载模型
  try {
    module_ = torch::jit::load(model_path);
    module_.to(torch::kCPU);
    module_.eval();
    RCLCPP_INFO(logger_, "Model loaded successfully from %s", model_path.c_str());
  } catch (const c10::Error& e) {
    RCLCPP_FATAL(logger_, "Failed to load model: %s", e.what());
    throw std::runtime_error("Failed to load PyTorch model");
  }

  // 创建订阅者
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SystemDefaultsQoS(), std::bind(&EndToEndController::scanCallback, this, std::placeholders::_1));

  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SystemDefaultsQoS(), std::bind(&EndToEndController::imuCallback, this, std::placeholders::_1));
}

void EndToEndController::activate() {
  RCLCPP_INFO(logger_, "EndToEndController activated");
}

void EndToEndController::deactivate() {
  RCLCPP_INFO(logger_, "EndToEndController deactivated");
}

void EndToEndController::cleanup() {
  scan_sub_.reset();
  imu_sub_.reset();
  RCLCPP_INFO(logger_, "EndToEndController cleaned up");
}

void EndToEndController::setPlan(const nav_msgs::msg::Path & path) {
  current_path_ = path;
  // 重置状态变量
  goal_reached_count_ = 0;
  // 保持上一次的有效目标，不重置 last_valid_goal_
  RCLCPP_INFO(logger_, "New path received with %zu poses", path.poses.size());
}

geometry_msgs::msg::TwistStamped EndToEndController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = node_.lock();
  auto clock = node->get_clock();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  
  if (current_path_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock, 1000, "Path is empty. Command zero velocity.");
    return cmd_vel;
  }

  const auto & goal_pose = current_path_.poses.back().pose;
  if (goal_checker->isGoalReached(pose.pose, goal_pose, velocity)) {
    RCLCPP_INFO(logger_, "Goal Checker reported goal is reached, stopping robot");
    return cmd_vel;
  }
  
  if (!current_scan_ || !current_imu_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock, 1000,
                         "Missing sensor data - scan: %s, imu: %s. Command zero velocity.",
                         current_scan_ ? "OK" : "NULL",
                         current_imu_ ? "OK" : "NULL");
    return cmd_vel;
  }
  
  try {
    std::vector<float> local_goal = computeLocalGoal(pose, current_path_);
    
    // 准备两个分离的输入张量
    torch::Tensor scan_tensor = prepareScanInput(*current_scan_);
    torch::Tensor aux_tensor = prepareAuxInput(*current_imu_, local_goal);
    
    // 调试信息
    RCLCPP_INFO_THROTTLE(logger_, *clock, 5000, 
                         "Input tensors - scan: [%ld, %ld], aux: [%ld, %ld]",
                         scan_tensor.size(0), scan_tensor.size(1),
                         aux_tensor.size(0), aux_tensor.size(1));
    
    torch::NoGradGuard no_grad;
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(scan_tensor);
    inputs.push_back(aux_tensor);
    torch::Tensor output = module_.forward(inputs).toTensor();
    
    output = output.squeeze().to(torch::kCPU);
    auto output_accessor = output.accessor<float, 1>();
    
    float raw_linear = output_accessor[0];
    float raw_angular = output_accessor[1];
    
    if (std::isnan(raw_linear) || std::isnan(raw_angular) ||
        std::isinf(raw_linear) || std::isinf(raw_angular)) {
      RCLCPP_WARN(logger_, "Model output contains NaN/Inf values, stopping robot.");
      return cmd_vel;
    }

    double linear_vel = static_cast<double>(raw_linear);
    double angular_vel = static_cast<double>(raw_angular);
    
    // 检测目标位置特征
    bool goal_behind = local_goal[0] < 0.0;
    bool large_lateral_offset = std::abs(local_goal[1]) > 0.3;  // 提高阈值，减少误判
    bool medium_lateral_offset = std::abs(local_goal[1]) > 0.15; // 提高中等偏移阈值
    
    // 更严格的直行检测 - 多重条件确保真正的直行
    bool is_straight = (std::abs(raw_angular) < 0.08 &&           // 模型输出角速度很小
                       std::abs(local_goal[1]) < 0.08 &&          // 横向偏移很小
                       local_goal[0] > 0.0 &&                     // 目标在前方
                       std::abs(local_goal[0]) > 0.2);            // 有足够的前向距离
    
    // 更严格的转弯检测，避免直行时的误判
    bool is_sharp_turn = std::abs(raw_angular) > 0.8 || large_lateral_offset || goal_behind;  // 提高阈值
    bool is_medium_turn = std::abs(raw_angular) > 0.4 || medium_lateral_offset;               // 提高中等转弯阈值
    
    // 分层角速度增强策略，重点处理直行稳定性
    double angular_boost = 1.0;
    if (is_straight) {
      // 直行时：严格抑制角速度，消除摇摆
      angular_vel = 0.0;  // 直行时完全清零角速度
      RCLCPP_INFO_THROTTLE(logger_, *clock, 3000, 
                           "STRAIGHT driving detected: zeroing angular velocity (raw=%.4f, offset=%.4f)", 
                           raw_angular, local_goal[1]);
    } else if (is_sharp_turn) {
      // 大幅转弯时的额外增强
      if (goal_behind) {
        angular_boost = 1.8;   // 目标在后方时更大增强
        linear_vel *= 0.3;        // 大幅降低线速度
      } else if (large_lateral_offset) {
        angular_boost = 1.5;   // 大横向偏移时的增强
        linear_vel *= 0.4;
      } else {
        angular_boost = 1.3;   // 普通急转弯
        linear_vel *= 0.5;
      }
      
      angular_vel *= angular_boost;
      RCLCPP_INFO_THROTTLE(logger_, *clock, 300, 
                           "SHARP turn: behind=%s, large_offset=%s, boost=%.1fx", 
                           goal_behind ? "Y" : "N", large_lateral_offset ? "Y" : "N", angular_boost);
    } else if (is_medium_turn) {
      // 中等转弯时的适度增强
      angular_boost = 1.2;
      linear_vel *= 0.7;  // 适度降低线速度
      angular_vel *= angular_boost;
      
      RCLCPP_INFO_THROTTLE(logger_, *clock, 500, 
                           "MEDIUM turn: offset=%.3f, boost=%.1fx", 
                           local_goal[1], angular_boost);
    } else {
      // 小转弯：保持适中的增强
      angular_boost = 1.1;  // 减小基础增强
      angular_vel *= angular_boost;
      RCLCPP_INFO_THROTTLE(logger_, *clock, 1000, 
                           "SMALL turn: angular=%.3f, offset=%.3f, boost=%.1fx", 
                           raw_angular, local_goal[1], angular_boost);
    }
    
    // 应用线速度限制
    linear_vel = std::clamp(linear_vel, -max_linear_velocity_, max_linear_velocity_);
    
    // 角速度平滑处理 - 添加历史记录和平滑
    angular_history_.push_back(angular_vel);
    if (angular_history_.size() > ANGULAR_HISTORY_SIZE) {
      angular_history_.erase(angular_history_.begin());
    }
    
    // 如果不是明确的转弯，使用平滑处理
    if (!is_sharp_turn && !is_medium_turn && angular_history_.size() >= 3) {
      // 计算历史角速度的平均值
      double avg_angular = 0.0;
      for (double hist_ang : angular_history_) {
        avg_angular += hist_ang;
      }
      avg_angular /= angular_history_.size();
      
      // 如果历史平均值很小且当前值也小，进一步抑制
      if (std::abs(avg_angular) < 0.1 && std::abs(angular_vel) < 0.2) {
        angular_vel *= 0.3;  // 大幅减小角速度
        RCLCPP_INFO_THROTTLE(logger_, *clock, 2000, 
                             "Angular smoothing: avg=%.4f, current=%.4f->%.4f", 
                             avg_angular, angular_vel/0.3, angular_vel);
      }
    }
    
    // 改进的角速度最小阈值处理，区分直行和转弯情况
    if (std::abs(angular_vel) > 1e-4) {
      if (is_straight) {
        // 直行时已经在上面清零了，这里不需要处理
      } else if (is_medium_turn || is_sharp_turn) {
        // 转弯时：确保有足够的角速度
        double min_angular_for_turn = 0.08;
        if (std::abs(angular_vel) < min_angular_for_turn) {
          angular_vel = std::copysign(min_angular_for_turn, angular_vel);
          RCLCPP_INFO_THROTTLE(logger_, *clock, 1000, "Applied min angular velocity for turn: %.3f", angular_vel);
        }
      }
    }
    
    // 记录当前角速度作为下次的历史
    last_angular_vel_ = angular_vel;

    if (std::abs(linear_vel) > 1e-4) {
      if (std::abs(linear_vel) < min_linear_velocity_) {
        linear_vel = std::copysign(min_linear_velocity_, linear_vel);
      }
    } else {
      linear_vel = 0.0;
    }
    
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;
    
    RCLCPP_INFO_THROTTLE(logger_, *clock, 400,
                         "Model: raw(%.3f,%.3f) -> cmd(%.3f,%.3f) | goal(%.3f,%.3f)",
                         raw_linear, raw_angular,
                         cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                         local_goal[0], local_goal[1]);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during model inference: %s. Stopping robot.", e.what());
  }
  
  return cmd_vel;
}

void EndToEndController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  RCLCPP_INFO(logger_, "Speed limit set to: %f (percentage: %s)",
              speed_limit, percentage ? "true" : "false");
}

void EndToEndController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  current_scan_ = msg;
}

void EndToEndController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  current_imu_ = msg;
}

std::vector<float> EndToEndController::computeLocalGoal(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & path)
{
  auto node = node_.lock();
  auto clock = node->get_clock();
  std::vector<float> local_goal = {0.0f, 0.0f};
  
  if (path.poses.empty()) {
    return last_valid_goal_;
  }
  
  // 检查是否接近终点
  const auto& final_pose = path.poses.back();
  double final_dx = final_pose.pose.position.x - current_pose.pose.position.x;
  double final_dy = final_pose.pose.position.y - current_pose.pose.position.y;
  double final_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy);
  
  // 如果非常接近终点，使用终点作为目标
  if (final_dist <= goal_tolerance_) {
    goal_reached_count_++;
    if (goal_reached_count_ > 3) {  // 连续多次确认到达终点
      RCLCPP_INFO_THROTTLE(logger_, *clock, 1000, "Very close to final goal (%.3f m), using final pose", final_dist);
      local_goal[0] = 0.0f;  // 给模型停止信号
      local_goal[1] = 0.0f;
      return local_goal;
    }
  } else {
    goal_reached_count_ = 0;
  }
  
  // 寻找最近点
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }
  
  // 动态调整前瞻距离，检测路径的弯曲程度
  double current_lookahead = lookahead_distance_;
  
  // 检测路径弯曲程度来调整前瞻距离
  if (closest_idx + 5 < path.poses.size() && path.poses.size() > 5) {
    size_t end_idx = std::min(closest_idx + 5, path.poses.size() - 1);
    if (end_idx > closest_idx + 2) {  // 确保有足够的点来计算曲率
      double curvature = calculatePathCurvature(path, closest_idx, end_idx);
      if (curvature > 0.5) {  // 检测到较大弯曲
        current_lookahead *= 0.7;  // 减小前瞻距离30%
        RCLCPP_INFO_THROTTLE(logger_, *clock, 1000, "High curvature detected (%.3f), reducing lookahead to %.3f", 
                             curvature, current_lookahead);
      }
    }
  }
  
  // 寻找前瞻目标点
  size_t target_idx = closest_idx;
  double best_dist = 0.0;
  
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
    double dist_from_robot = std::sqrt(dx * dx + dy * dy);
    
    if (dist_from_robot >= min_lookahead_distance_) {
      if (dist_from_robot >= current_lookahead) {
        target_idx = i;
        best_dist = dist_from_robot;
        break;
      } else if (dist_from_robot > best_dist) {
        target_idx = i;
        best_dist = dist_from_robot;
      }
    }
  }
  
  // 如果没有找到合适的前瞻点，使用路径中剩余的最远点
  if (target_idx == closest_idx && closest_idx < path.poses.size() - 1) {
    target_idx = std::min(closest_idx + 3, path.poses.size() - 1);
  }
  
  const auto& target_pose_global = path.poses[target_idx];
  
  geometry_msgs::msg::PoseStamped goal_pose_local;
  try {
    auto transform = tf_->lookupTransform(
        costmap_ros_->getBaseFrameID(),
        path.header.frame_id,
        tf2::TimePointZero);
    tf2::doTransform(target_pose_global, goal_pose_local, transform);

        // 改进的后方目标处理逻辑
    if (goal_pose_local.pose.position.x < -0.05) {  // 更严格的后方判断
      // 尝试寻找前方的替代目标点
      bool found_forward_goal = false;
      for (size_t i = target_idx + 1; i < std::min(target_idx + 5, path.poses.size()); ++i) {
        geometry_msgs::msg::PoseStamped alt_goal_local;
        tf2::doTransform(path.poses[i], alt_goal_local, transform);
        
        if (alt_goal_local.pose.position.x >= 0.0) {
          local_goal[0] = static_cast<float>(alt_goal_local.pose.position.x);
          local_goal[1] = static_cast<float>(alt_goal_local.pose.position.y);
          found_forward_goal = true;
          RCLCPP_INFO_THROTTLE(logger_, *clock, 1000,
                               "Found alternative forward goal at index %zu: (%.3f, %.3f)", 
                               i, local_goal[0], local_goal[1]);
          break;
        }
      }
      
      if (!found_forward_goal) {
        // 如果找不到前方目标，使用上一次的有效目标
        if (std::abs(last_valid_goal_[0]) > 1e-4 || std::abs(last_valid_goal_[1]) > 1e-4) {
          local_goal = last_valid_goal_;
          RCLCPP_WARN_THROTTLE(logger_, *clock, 1000,
                               "No forward goal found, using last valid goal: (%.3f, %.3f)",
                               local_goal[0], local_goal[1]);
        } else {
          // 最后的选择：给一个小的前方目标
          local_goal[0] = 0.1f;
          local_goal[1] = 0.0f;
          RCLCPP_WARN_THROTTLE(logger_, *clock, 1000,
                               "Using default forward goal: (%.3f, %.3f)", local_goal[0], local_goal[1]);
        }
      }
    } else {
      // 目标在前方，正常使用
      local_goal[0] = static_cast<float>(goal_pose_local.pose.position.x);
      local_goal[1] = static_cast<float>(goal_pose_local.pose.position.y);
      
      // 更新最后的有效目标
      last_valid_goal_ = local_goal;
    }

  } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Could not transform goal from %s to %s: %s",
                   path.header.frame_id.c_str(), costmap_ros_->getBaseFrameID().c_str(), ex.what());
      return last_valid_goal_;
  }
  
  RCLCPP_INFO_THROTTLE(logger_, *clock, 1000,
                       "Path info: closest_idx=%zu, target_idx=%zu/%zu. Local Goal: (%.3f, %.3f)",
                       closest_idx, target_idx, path.poses.size()-1, local_goal[0], local_goal[1]);
  
  return local_goal;
}

torch::Tensor EndToEndController::prepareScanInput(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<float> scan_data;
  scan_data.reserve(scan.ranges.size());

  for (float range : scan.ranges) {
    if (std::isnan(range) || std::isinf(range)) {
      scan_data.push_back(10.0f);  // 使用与训练时相同的值替换inf
    } else {
      scan_data.push_back(range);
    }
  }
  
  // 创建张量并添加必要的维度 - 模型期望 (batch, channels, length)
  torch::Tensor tensor = torch::tensor(scan_data, torch::dtype(torch::kFloat32));
  tensor = tensor.unsqueeze(0).unsqueeze(0);  // 添加batch维度和channel维度: (1, 1, num_points)
  return tensor.to(torch::kCPU);
}

torch::Tensor EndToEndController::prepareAuxInput(
  const sensor_msgs::msg::Imu & imu,
  const std::vector<float> & local_goal)
{
  std::vector<float> aux_data;
  aux_data.reserve(4); // IMU角速度 + 线加速度 + 目标x + 目标y
  
  aux_data.push_back(static_cast<float>(imu.angular_velocity.z));
  aux_data.push_back(static_cast<float>(imu.linear_acceleration.x));
  aux_data.push_back(local_goal[0]);
  aux_data.push_back(local_goal[1]);
  
  // 直接从数据创建张量，避免内存管理问题
  torch::Tensor tensor = torch::tensor(aux_data, torch::dtype(torch::kFloat32));
  return tensor.unsqueeze(0).to(torch::kCPU);  // 添加batch维度
}

double EndToEndController::calculatePathCurvature(
  const nav_msgs::msg::Path & path, 
  size_t start_idx, 
  size_t end_idx)
{
  if (end_idx <= start_idx + 2 || end_idx >= path.poses.size() || path.poses.size() < 3) {
    return 0.0;
  }
  
  // 计算路径段的方向变化
  double total_angle_change = 0.0;
  double total_distance = 0.0;
  
  for (size_t i = start_idx; i < end_idx - 1 && i + 2 < path.poses.size(); ++i) {
    const auto& p1 = path.poses[i].pose.position;
    const auto& p2 = path.poses[i + 1].pose.position;
    const auto& p3 = path.poses[i + 2].pose.position;
    
    // 计算两个连续线段的方向
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double angle1 = std::atan2(dy1, dx1);
    double angle2 = std::atan2(dy2, dx2);
    
    // 计算角度变化（处理角度环绕）
    double angle_diff = angle2 - angle1;
    while (angle_diff > 3.14159265359) angle_diff -= 2 * 3.14159265359;
    while (angle_diff < -3.14159265359) angle_diff += 2 * 3.14159265359;
    
    total_angle_change += std::abs(angle_diff);
    
    // 计算距离
    double dist = std::sqrt(dx1 * dx1 + dy1 * dy1);
    total_distance += dist;
  }
  
  // 返回曲率（角度变化/距离）
  return total_distance > 1e-6 ? total_angle_change / total_distance : 0.0;
}

} // namespace zzzplan

PLUGINLIB_EXPORT_CLASS(zzzplan::EndToEndController, nav2_core::Controller)