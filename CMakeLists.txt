cmake_minimum_required(VERSION 3.8)
project(timestamp_overwrite)

  # Set compile options
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(overwrite_pointcloud_timestamp_lib SHARED
 src/overwrite_pointcloud_timestamp.cpp
)
target_include_directories(overwrite_pointcloud_timestamp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(overwrite_pointcloud_timestamp_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rclcpp_components_register_node(overwrite_pointcloud_timestamp_lib
  PLUGIN "timestamp_overwrite::OverwritePointCloudTimeStamp"
  EXECUTABLE overwrite_pointcloud_timestamp
)

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

ament_auto_package(INSTALL_TO_SHARE
)
