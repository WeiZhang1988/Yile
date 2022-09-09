cmake_minimum_required(VERSION 3.16)
project(MyProject VERSION 1.0.0)
set (CMAKE_CXX_STANDARD 17)

set(BOOST_ROOT "./dep/boost_1_80_0")
set(BOOST_LIBRARYDIR "./dep/boost_1_80_0/libs")
set(BOOST_INCLUDE_DIR "./dep/boost_1_80_0/boost")

find_package(Boost 1.80.0)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
endif()

add_subdirectory(src)
install(DIRECTORY data DESTINATION ${PROJECT_SOURCE_DIR}/install)
