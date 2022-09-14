add_library(
    models SHARED
    components/common.cpp
    components/wheel_disk.cpp
    subsystems/subsystem_whl_4disk.cpp
    interfaces/interface_whl_4disk.cpp
    systems/system_whl_4disk.cpp
)


target_include_directories(models PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

install(FILES components/common.hpp
    		  components/wheel_disk.hpp
    		  subsystems/subsystem_whl_4disk.hpp
    		  interfaces/interface_whl_4disk.hpp
    		  systems/system_whl_4disk.hpp
			  DESTINATION ${PROJECT_SOURCE_DIR}/install/include)
#install(TARGETS models ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/install/lib)
#install(TARGETS models LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/share)
install(TARGETS models LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install)
