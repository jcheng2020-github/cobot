cmake_minimum_required(VERSION 2.8.3)
project(gripper_hardware_interface)

if(WIN32)
    macro(get_WIN32_WINNT version)
        if(CMAKE_SYSTEM_VERSION)
            set(ver ${CMAKE_SYSTEM_VERSION})
            string(REGEX MATCH "^([0-9]+).([0-9])" ver ${ver})
            string(REGEX MATCH "^([0-9]+)" verMajor ${ver})
            # Check for Windows 10, b/c we'll need to convert to hex 'A'.
            if("${verMajor}" MATCHES "10")
                set(verMajor "A")
                string(REGEX REPLACE "^([0-9]+)" ${verMajor} ver ${ver})
            endif()
            # Remove all remaining '.' characters.
            string(REPLACE "." "" ver ${ver})
            # Prepend each digit with a zero.
            string(REGEX REPLACE "([0-9A-Z])" "0\\1" ver ${ver})
            set(${version} "0x${ver}")
        endif()
    endmacro()

    get_WIN32_WINNT(ver)
    add_definitions(-D_WIN32_WINNT=${ver})
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  urdf
  joint_limits_interface
  gripper_hardware_drivers
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    gripper_hardware_interface
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    urdf
    joint_limits_interface
    gripper_hardware_drivers
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(gripper_hardware_interface src/gripper_hardware_interface.cpp)
target_link_libraries(gripper_hardware_interface
	${catkin_LIBRARIES}
)

add_executable(gripper_hardware_interface_node src/gripper_hardware_interface_node.cpp)
add_dependencies(gripper_hardware_interface_node gripper_hardware_interface)

target_link_libraries(gripper_hardware_interface_node
	gripper_hardware_interface
	${catkin_LIBRARIES}
)
