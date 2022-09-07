add_executable(vehicle_dynamics_test 
			   test.cpp 
			   simulators/simulator_whl_disk_tir_fiala.cpp
			   simulators/simulator_tir_4fiala.cpp
			   simulators/simulator_sus_2ind.cpp
			   simulators/simulator_vehicle_body.cpp
			   simulators/simulator_whl_4disk.cpp)
target_link_libraries(vehicle_dynamics_test PRIVATE models)

#install(TARGETS vehicle_dynamics_test DESTINATION ${PROJECT_SOURCE_DIR}/install/exe)
install(TARGETS vehicle_dynamics_test DESTINATION ${PROJECT_SOURCE_DIR}/install)
