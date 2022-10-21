# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/houtest)

# add_executable(vehicle_dynamics_test 
# 			   test.cpp 
# 			   simulators/simulator_whl_4disk.cpp
# 			   simulators/simulator_tir_4fiala.cpp
# 			   simulators/simulator_sus_2ind.cpp
# 			   simulators/simulator_vehicle_body.cpp
# 			   simulators/simulator_chassis_2ind_disk_fiala.cpp
# 			   simulators/simulator_wheel_tire_4disk_fiala.cpp
# 			   simulators/simulator_sus_wheel_tire_2ind_disk_fiala.cpp
# 			   )
# target_link_libraries(vehicle_dynamics_test PRIVATE models
# 													pthread)

# install(TARGETS vehicle_dynamics_test DESTINATION ${PROJECT_SOURCE_DIR}/install)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/houtest)
add_executable(test 
			   test.cpp 
			   simulators/simulator_whl_4disk.cpp
 			   simulators/simulator_tir_4fiala.cpp
 			   simulators/simulator_sus_2ind.cpp
 			   simulators/simulator_vehicle_body.cpp
 			   simulators/simulator_chassis_2ind_disk_fiala.cpp
 			   simulators/simulator_wheel_tire_4disk_fiala.cpp
 			   simulators/simulator_sus_wheel_tire_2ind_disk_fiala.cpp
			   simulators/simulator_pass14dof.cpp
			   )
add_executable(udp_client
			   udp_client.cpp	
			  )			
target_link_libraries(test PRIVATE models pthread)
target_link_libraries(udp_client PRIVATE models pthread)
target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/simulators)
install(TARGETS test udp_client DESTINATION ${PROJECT_SOURCE_DIR}/install)

