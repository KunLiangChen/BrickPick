cmake_minimum_required(VERSION 3.8)
project(brickpick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(robomaster_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# 🔹 新增：BehaviorTree 相关依赖 🔹
find_package(rclcpp REQUIRED)  
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcpputils REQUIRED)
# 安装 Python 脚本，使其可以通过 ros2 run 运行
install(PROGRAMS
  brickpick/arm_preset_node.py
  brickpick/vision_node.py
  brickpick/teleop_keyboard_node.py
  brickpick/approach_node.py
  brickpick/find_node.py
  brickpick/image_capture_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 config 目录（参数文件）
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

#Install model file
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}/)

# # 🔹 新增：编译 BehaviorTree 执行器节点 🔹
# add_executable(bt_executor
#   src/bt_executor.cpp
#   src/find_action.cpp
#   src/approach_action.cpp
#   src/arm_action.cpp
# )

# # 包含头文件目录（含 BT 和当前包）
# target_include_directories(bt_executor PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${behaviortree_cpp_INCLUDE_DIRS}
# )

# # 链接库
# ament_target_dependencies(bt_executor
#   rclcpp
#   std_srvs
#   action_msgs
#   behaviortree_cpp
#   rcpputils
# )

# # 🔹 关键：链接行为树插件库（防止运行时找不到符号） 🔹
# target_link_libraries(bt_executor
#   behaviortree_cpp::behaviortree_cpp
# )

# 安装可执行文件
# install(TARGETS bt_executor
#   DESTINATION lib/${PROJECT_NAME}
# )
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
#waiting for launch file
ament_package()
