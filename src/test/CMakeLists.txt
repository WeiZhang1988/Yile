set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(vehicle_dynamics_test 
			   test.cpp 
			   simulators/simulator_whl_4disk.cpp
			   simulators/simulator_tir_4fiala.cpp)
target_link_libraries(vehicle_dynamics_test PRIVATE models
													pthread)

install(TARGETS vehicle_dynamics_test DESTINATION ${PROJECT_SOURCE_DIR}/install)
