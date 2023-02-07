Vehicle dynamics library in C++ 

dependent on boost-1.80.0 library

demo with carla

[!["co-sim with carla"](./pics/demo_with_carla.gif "demo with carla")](https://www.youtube.com/watch?v=cFAeuJHYPG4)

simulation results compared to simulink model of vehicle dynamics blocks

blue lines are our results and orange dash lines are simulink results

note: SIMULINK VEHICLE DYNAMICS BLOCKS (at least in version 2021 and before) have ERRORS when the car is drving backward.

The car turns without steering input when driving backward

entire chassis sim result

!["entire chassis sim result"](./pics/chassis_cg_diff.png "entire chassis sim result")

vehicle body sim result
!["vehicle body sim result"](./pics/veh_cg_diff.png "vehicle body sim result")

suspension sim result
!["suspension sim result"](./pics/sus.png "suspension sim result")

wheel tire sim result
!["wheel tire sim result"](./pics/whl_tir_disk.png "wheel tire sim result")
