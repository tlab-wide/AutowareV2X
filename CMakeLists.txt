cmake_minimum_required(VERSION 3.5)
project(autoware_v2x)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
set(VANETZA_INSTALL ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
find_package(Vanetza REQUIRED)
find_package(GeographicLib 1.37 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
)

ament_auto_add_library(autoware_v2x SHARED
  src/v2x_node.cpp
  src/v2x_app.cpp
  src/application.cpp
  src/cpm_application.cpp
  src/ethernet_device.cpp
  src/link_layer.cpp
  src/raw_socket_link.cpp
  src/router_context.cpp
  src/dcc_passthrough.cpp
  src/time_trigger.cpp
  src/positioning.cpp
  src/security.cpp
  src/tcpip_application.cpp
)

target_link_libraries(autoware_v2x Vanetza::vanetza ${GeographicLib_LIBRARIES} Boost::thread sqlite3 ${PCL_LIBRARIES})

rclcpp_components_register_node(autoware_v2x
  PLUGIN "v2x::V2XNode"
  EXECUTABLE autoware_v2x_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)