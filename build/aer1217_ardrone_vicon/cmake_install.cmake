# Install script for directory: /home/thurteen/aer1217/labs/src/aer1217_ardrone_vicon

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/thurteen/aer1217/labs/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aer1217_ardrone_vicon/msg" TYPE FILE FILES
    "/home/thurteen/aer1217/labs/src/aer1217_ardrone_vicon/msg/MotorCommands.msg"
    "/home/thurteen/aer1217/labs/src/aer1217_ardrone_vicon/msg/GazeboState.msg"
    "/home/thurteen/aer1217/labs/src/aer1217_ardrone_vicon/msg/DesiredStateMsg.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aer1217_ardrone_vicon/cmake" TYPE FILE FILES "/home/thurteen/aer1217/labs/build/aer1217_ardrone_vicon/catkin_generated/installspace/aer1217_ardrone_vicon-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/thurteen/aer1217/labs/devel/include/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/thurteen/aer1217/labs/devel/share/roseus/ros/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/thurteen/aer1217/labs/devel/share/common-lisp/ros/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/thurteen/aer1217/labs/devel/share/gennodejs/ros/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/thurteen/aer1217/labs/devel/lib/python2.7/dist-packages/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/thurteen/aer1217/labs/devel/lib/python2.7/dist-packages/aer1217_ardrone_vicon")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/thurteen/aer1217/labs/build/aer1217_ardrone_vicon/catkin_generated/installspace/aer1217_ardrone_vicon.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aer1217_ardrone_vicon/cmake" TYPE FILE FILES "/home/thurteen/aer1217/labs/build/aer1217_ardrone_vicon/catkin_generated/installspace/aer1217_ardrone_vicon-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aer1217_ardrone_vicon/cmake" TYPE FILE FILES
    "/home/thurteen/aer1217/labs/build/aer1217_ardrone_vicon/catkin_generated/installspace/aer1217_ardrone_viconConfig.cmake"
    "/home/thurteen/aer1217/labs/build/aer1217_ardrone_vicon/catkin_generated/installspace/aer1217_ardrone_viconConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aer1217_ardrone_vicon" TYPE FILE FILES "/home/thurteen/aer1217/labs/src/aer1217_ardrone_vicon/package.xml")
endif()

