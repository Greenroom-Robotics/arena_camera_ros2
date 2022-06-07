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
find_package(arena_sdk_vendor REQUIRED)

# -----------------------------------------------------------------------------
#
# ROS2 RUN ARENA_CAMERA_NODE <>
#
# -----------------------------------------------------------------------------

set(target_name camera_node)

# CPP DEPS
add_executable(${target_name}
    src/ArenaCameraNode.cpp
    src/light_arena/deviceinfo_helper.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
    src/rclcpp_adapter/quilty_of_service_translation.cpp
)

# HEADERS DEP
#target_include_directories(${target_name}
  #PRIVATE
    #${arena_sdk_INCLUDE_DIRS}
#  PUBLIC
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #$<INSTALL_INTERFACE:include>
#)

# CPP -> .a file
# creates :
# - build/arena_camera_node/CMakeFiles/arena_camera_node.dir and sub dirs
# - build/arena_camera_node/libarena_camera_node.a
# add_library(${PROJECT_NAME} src/main.cpp)

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

ament_target_dependencies(${target_name}  rclcpp 
                                          std_msgs 
                                          std_srvs
                                          sensor_msgs
                                          arena_sdk_vendor
                                          )


install(TARGETS ${target_name} 
    DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#
# ROS2 RUN TRIGGER_IMAGE <>
#
# -----------------------------------------------------------------------------

set(trigger_image trigger_image)

# CPP -> EXECUTABLE
add_executable(${trigger_image} src/trigger_image_client.cpp)

# CPP -> .a file

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# ROS DEP
# - this way it will find our header libs automatically
# - for ArenAsdk the cmake way is used no ament
ament_target_dependencies(${trigger_image}  rclcpp 
                                          std_srvs arena_sdk_vendor)

# HEADERS DEP
target_include_directories(${trigger_image}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# MV EXEC to lib dir
install(TARGETS
  ${trigger_image} 
    DESTINATION lib/${PROJECT_NAME}
)
# -----------------------------------------------------------------------------

# Include launch file directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# TESTING ---------------------------------------------------------------------

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

#message(FATAL_ERROR ********************************************)