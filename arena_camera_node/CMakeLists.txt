cmake_minimum_required(VERSION 3.15)

project(arena_camera_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# linked GenICam hearders give warnings No such file or directory
	# GENICAM_USER_ACCEPTS_ANY_COMPILER flag need tro be defined 
	# 
	#add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(arena_sdk_vendor REQUIRED)

add_executable(camera_node
    src/ArenaCameraNode.cpp
    src/light_arena/deviceinfo_helper.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
    src/rclcpp_adapter/quilty_of_service_translation.cpp
)

ament_target_dependencies(camera_node
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        camera_calibration_parsers
        arena_sdk_vendor
)


install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME}
)

add_executable(trigger_image
        src/trigger_image_client.cpp)

ament_target_dependencies(trigger_image
        rclcpp
        std_srvs
)

# HEADERS DEP
target_include_directories(trigger_image
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS trigger_image
        DESTINATION lib/${PROJECT_NAME}
)

# Include launch file directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
