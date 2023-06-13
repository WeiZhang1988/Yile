# Install script for directory: /home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/common.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/wheel_disk.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/tire_fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/sus_ind_2tracks.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/vehicle_body.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/subsystem_whl_4disk.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/subsystem_tir_4fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/subsystem_sus_2ind.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/interface_whl_4disk.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/interface_tir_4fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/interface_sus_2ind.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/interface_vehicle_body.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/interface_chassis_2ind_disk_fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_whl_4disk.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_tir_4fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_sus_2ind.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_vehicle_body.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_chassis_2ind_disk_fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_wheel_tire_4disk_fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/system_sus_wheel_tire_2ind_disk_fiala.hpp;/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include/yile.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/include" TYPE FILE FILES
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/components/common.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/components/wheel_disk.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/components/tire_fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/components/sus_ind_2tracks.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/components/vehicle_body.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/subsystems/subsystem_whl_4disk.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/subsystems/subsystem_tir_4fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/subsystems/subsystem_sus_2ind.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/interfaces/interface_whl_4disk.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/interfaces/interface_tir_4fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/interfaces/interface_sus_2ind.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/interfaces/interface_vehicle_body.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/interfaces/interface_chassis_2ind_disk_fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_whl_4disk.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_tir_4fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_sus_2ind.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_vehicle_body.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_chassis_2ind_disk_fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_wheel_tire_4disk_fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/systems/system_sus_wheel_tire_2ind_disk_fiala.hpp"
    "/home/ubuntu/Documents/PROJECT_YILE_GRADE/src/models/yile.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install" TYPE SHARED_LIBRARY FILES "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/models/libmodels.so")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/libmodels.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

