cmake_minimum_required(VERSION 3.5)

project(phase_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_DIRECTORIES_BEFORE ON)

set(LIBRARY_TARGET_NAME phase_manager) # set library name
set(phase_manager_SRC
    src/phase_manager.cpp
    src/timeline.cpp
    src/phase.cpp
)
set(phase_manager_ros_SRC
    src/ros_server_class.cpp
)

option(WITH_ROS_COMPONENTS "Whether to install ros-related components (for debugging) or not" FALSE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(DEFINED ENV{CONDA_PREFIX})
  set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX}/ CACHE PATH "bindings install prefix" FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED)

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
)

if(WITH_ROS_COMPONENTS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
  add_message_files(
    FILES
    Timeline.msg
    TimelineArray.msg
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package()
  
endif()

if(WITH_ROS_COMPONENTS)
  add_library(${LIBRARY_TARGET_NAME} SHARED
    ${phase_manager_SRC}
    ${phase_manager_ros_SRC}
  )
  target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
  )
else()
  add_library(${LIBRARY_TARGET_NAME} SHARED
    ${phase_manager_SRC}
  )
endif()

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

add_subdirectory(bindings/python/)
add_subdirectory(tests/)
