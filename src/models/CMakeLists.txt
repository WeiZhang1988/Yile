add_library(
    models SHARED
    components/common.cpp
    components/wheel_disk.cpp
    components/tire_fiala.cpp
    components/sus_ind_2tracks.cpp
    components/vehicle_body.cpp
    subsystems/subsystem_whl_4disk.cpp
    subsystems/subsystem_tir_4fiala.cpp
    subsystems/subsystem_sus_2ind.cpp
    interfaces/interface_whl_4disk.cpp
    interfaces/interface_tir_4fiala.cpp
    interfaces/interface_sus_2ind.cpp
    interfaces/interface_vehicle_body.cpp
    interfaces/interface_chassis_2ind_disk_fiala.cpp
    systems/system_whl_4disk.cpp
    systems/system_tir_4fiala.cpp
    systems/system_sus_2ind.cpp
    systems/system_vehicle_body.cpp
    systems/system_chassis_2ind_disk_fiala.cpp
    systems/system_wheel_tire_4disk_fiala.cpp
)


target_include_directories(models PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

install(FILES components/common.hpp
    		  components/wheel_disk.hpp
    		  components/tire_fiala.hpp
    		  components/sus_ind_2tracks.hpp
    		  components/vehicle_body.hpp
    		  subsystems/subsystem_whl_4disk.hpp
    		  subsystems/subsystem_tir_4fiala.hpp
    		  subsystems/subsystem_sus_2ind.hpp
    		  interfaces/interface_whl_4disk.hpp
    		  interfaces/interface_tir_4fiala.hpp
    		  interfaces/interface_sus_2ind.hpp
    		  interfaces/interface_vehicle_body.hpp
    		  interfaces/interface_chassis_2ind_disk_fiala.hpp
    		  systems/system_whl_4disk.hpp
    		  systems/system_tir_4fiala.hpp
    		  systems/system_sus_2ind.hpp
    		  systems/system_vehicle_body.hpp
    		  systems/system_chassis_2ind_disk_fiala.hpp
              systems/system_wheel_tire_4disk_fiala.hpp
			  DESTINATION ${PROJECT_SOURCE_DIR}/install/include)
#install(TARGETS models ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/install/lib)
#install(TARGETS models LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/share)
install(TARGETS models LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install)
