# Install script for directory: /home/ubuntu/Documents/PROJECT_YILE_GRADE/src/demo

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
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install" TYPE EXECUTABLE FILES "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/demo/demo_server")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server"
         OLD_RPATH "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/models:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_server")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install" TYPE EXECUTABLE FILES "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/demo/demo_client")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client"
         OLD_RPATH "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/models:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo_client")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/Documents/PROJECT_YILE_GRADE/install" TYPE EXECUTABLE FILES "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/demo/demo")
  if(EXISTS "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo"
         OLD_RPATH "/home/ubuntu/Documents/PROJECT_YILE_GRADE/build/src/models:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ubuntu/Documents/PROJECT_YILE_GRADE/install/demo")
    endif()
  endif()
endif()

