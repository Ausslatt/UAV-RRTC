# UAV RRT-Connect Planner (RotorS)

This project adds a custom **RRT-Connect** planner in the **RotorS** simulator (ROS + Gazebo).

1. You set up a VM and get RotorS working by following the official tutorial.
2. Then replace your `catkin_ws` with the once from this project.
3. source your ws
4. run 

---

## 1. Set up the VM and RotorS

1. Create an Ubuntu 16.04 VM (VirtualBox).
	- My resources: 10gb ram, 10 cores, 75mb video 
2. Follow the **RotorS GitHub README / tutorial** to:
   - Install ROS
   - Create a `catkin_ws`
   - Clone `rotors_simulator`
   - Build it
   - Run at least one RotorS example to confirm everything works.
   - Follow the instructions for Ubuntu 16.04 only

Once you can run a RotorS example world and see the Firefly fly you should be ready. 

---

## 2. Use this `catkin_ws`

replace the `catkin_ws` folder from the RotorS tutorial with the one from this project.

Make sure `~/catkin_ws/src/rotors_gazebo/world/test_city_RRTC.world`, `~/catkin_ws/src/uav_RRTC_planner/RRTC_planner_node.py` and  `~/catkin_ws/src/uav_RRTC_planner/launch/RRTC_example.launch` exist after you copy the project ws over. 

```
source ~/.bashrc
catkin_build
```

Ensure the build was a success and run.

```
roslaunch uav_RRTC_planner RRTC_example.launch   mav_name:=firefly   world_name:=test_city_RRTC
```

Gazebo should launch and you should see the test city and the UAV at the x_start location defined in the RRTC_example.lauch file. Press play and the MAV will use the RRTC trajectory to fly to x_goal. To try out different goals/starts you can edit the RRTC_example file. 






@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}





