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
find_package(Boost COMPONENTS thread program_options REQUIRED)
find_package(etsi_its_cam_ts_coding REQUIRED)
find_package(etsi_its_cam_ts_conversion REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/src
  ../etsi_its_messages/etsi_its_msgs_utils/include
)

set(SOURCES
  src/v2x_node.cpp
  src/v2x_app.cpp
  src/application.cpp
  src/cpm_application.cpp
  src/cam_application.cpp
  src/cube_evk_link.cpp
  src/ethernet_device.cpp
  src/link_layer.cpp
  src/raw_socket_link.cpp
  src/router_context.cpp
  src/dcc_passthrough.cpp
  src/time_trigger.cpp
  src/positioning.cpp
  src/security.cpp
)

if(DEFINED BUILD_COHDA)
  set(SEARCH_PATHS
    "/v2x/cohda/llc"
    "/usr/include/linux/cohda/llc"
    "/usr/include/cohda/llc"
  )
  find_file(COHDA_LLC_API_HEADER llc-api.h
    PATHS ${SEARCH_PATHS}
    CMAKE_FIND_ROOT_PATH_BOTH
    DOC "Cohda LLC API header"
  )
  mark_as_advanced(COHDA_LLC_API_HEADER)
  if(COHDA_LLC_API_HEADER)
    get_filename_component(COHDA_LLC_INCLUDE_DIR ${COHDA_LLC_API_HEADER} DIRECTORY)
    include_directories(
      include
      ${CMAKE_CURRENT_BINARY_DIR}/src
      ${COHDA_LLC_INCLUDE_DIR}
    )
    list(APPEND SOURCES src/cohda_link.cpp)
    add_compile_definitions(BUILD_COHDA)
  else()
    string(REPLACE ";" " " SEARCH_PATHS_STR "${SEARCH_PATHS}")
    message(WARNING "Cohda LLC API header not found, could not enable Cohda support.\nSearched in: ${SEARCH_PATHS_STR}")
  endif()
endif()

ament_auto_add_library(autoware_v2x SHARED ${SOURCES})

protobuf_generate(TARGET autoware_v2x PROTOS src/cube_evk_radio.proto)

target_link_libraries(autoware_v2x Vanetza::vanetza ${GeographicLib_LIBRARIES} Boost::thread Boost::program_options sqlite3 etsi_its_cam_ts_coding::etsi_its_cam_ts_coding etsi_its_cam_ts_conversion::etsi_its_cam_ts_conversion protobuf::libprotobuf)

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
