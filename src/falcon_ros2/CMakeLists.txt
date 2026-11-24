cmake_minimum_required(VERSION 3.8)
project(falcon_ros2)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# libnifalcon installs to /usr/local by default; a no-sudo install to ~/.local also works
find_path(NIFALCON_INCLUDE_DIR falcon/core/FalconDevice.h PATHS /usr/local/include $ENV{HOME}/.local/include)
find_library(NIFALCON_LIB nifalcon PATHS /usr/local/lib $ENV{HOME}/.local/lib)
find_library(USB_LIB usb-1.0)
if(NOT NIFALCON_INCLUDE_DIR OR NOT NIFALCON_LIB)
  message(FATAL_ERROR "libnifalcon not found - build it and run 'sudo make install && sudo ldconfig'")
endif()
# Keep the libnifalcon path in the installed binary's RPATH so ~/.local needs no ldconfig
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(falcon_node src/falcon_node.cpp)
target_include_directories(falcon_node PRIVATE ${NIFALCON_INCLUDE_DIR})
target_link_libraries(falcon_node ${NIFALCON_LIB} ${USB_LIB})
ament_target_dependencies(falcon_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS falcon_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY udev launch worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/falcon_ur_teleop.py
  scripts/admittance_insert.py
  scripts/contact_limit.py
  scripts/record_trial.sh
  scripts/analyze_trial.py
  scripts/fit_admittance.py
  scripts/move_to_ready.py
  scripts/position_hold_demo.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contact_limit test/test_contact_limit.py)
endif()

ament_package()
