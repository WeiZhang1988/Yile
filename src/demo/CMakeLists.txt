#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install)
add_executable(demo_server 
			   demo_server.cpp 
			   simulators/simulator_whl_4disk.cpp
 			   simulators/simulator_tir_4fiala.cpp
 			   simulators/simulator_sus_2ind.cpp
 			   simulators/simulator_vehicle_body.cpp
 			   simulators/simulator_chassis_2ind_disk_fiala.cpp
 			   simulators/simulator_wheel_tire_4disk_fiala.cpp
 			   simulators/simulator_sus_wheel_tire_2ind_disk_fiala.cpp
			   simulators/simulator_pass14dof.cpp
			   )
add_executable(demo_client
			   demo_client.cpp	
			  )		
add_executable(demo
			   demo.cpp	
			  )		
target_link_libraries(demo_server PRIVATE models pthread)
target_link_libraries(demo_client PRIVATE models pthread)
target_link_libraries(demo PRIVATE models pthread)
target_include_directories(demo_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/simulators)
install(TARGETS demo_server demo_client demo DESTINATION ${PROJECT_SOURCE_DIR}/install)
