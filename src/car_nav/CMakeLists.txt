cmake_minimum_required(VERSION 3.8)
project(car_nav)

# ===== 基础编译设置 =====
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 设置 Torch 路径
set(Torch_DIR ~/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

# 设置 Torch 库路径
set(TORCH_LIB_DIR ~/libtorch/lib)
link_directories(${TORCH_LIB_DIR})
# ===== 插件库 =====
add_library(zzzplan SHARED
  planner_config/plan.cpp
)

target_include_directories(zzzplan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner_config>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
) 

ament_target_dependencies(zzzplan
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 链接Torch库
target_link_libraries(zzzplan ${TORCH_LIBRARIES})

# 设置 RPATH 以便运行时能找到 PyTorch 库
set_target_properties(zzzplan PROPERTIES
  INSTALL_RPATH "${TORCH_LIB_DIR};/home/zzz/.local/lib/python3.10/site-packages/torch/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)
# 导出插件（必需）
pluginlib_export_plugin_description_file(nav2_core planner_config/plan.xml)

# ===== 安装：库与头文件 =====
install(TARGETS zzzplan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 把 plan.hpp 装到 include/car_nav/planner_config/
install(FILES planner_config/plan.hpp
  DESTINATION include/${PROJECT_NAME}/planner_config
)

# XML 文件会被 pluginlib_export_plugin_description_file 安装，
# 这里可选再安装一份到 share 下
install(FILES planner_config/plan.xml
  DESTINATION share/${PROJECT_NAME}/planner_config
)

# ===== 安装：资源（按你的包结构调整）=====
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY maps-1
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# ===== 安装：脚本（Python 可执行）=====
install(PROGRAMS
  scripts/odom_bridge.py
  scripts/steering_from_odom.py
  scripts/wheels_from_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===== 导出 =====
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ===== 测试 / Lint =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
