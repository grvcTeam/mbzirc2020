# mbzirc2020

Catkin repository to store ROS packages for the MBZIRC2020

## Dependencies

A list of required ROS default packages are provided in the root of this repo ([requirements.txt](requirements.txt)). For installing them:
```
sudo apt-get install $(cat requirements.txt)
```

Additionally:
```
sudo pip install crcmod
```

Finally, be sure to clone [grvc-ual](https://github.com/grvcTeam/grvc-ual) and checkout the **mbzirc2020** tag. Also get an updated **master**-branch clone of [grvc-utils](https://github.com/grvcTeam/grvc-utils).


## Run instructions

### Challenge 2
To run the simulation:
```
roslaunch mbzirc_launchers c2_sim.launch
```  

If Gazebo's Real Time Factor falls far below 1.0 and proper flight control is not possible, consider using the light simulation version (no SITL):
```
roslaunch mbzirc_launchers c2_sim_light.launch
```  

For visualization:
```
roslaunch mbzirc_visualization visualization.launch
```

---
### Challenge 3
To run the simulation of facade fires extinction:
```
roslaunch mbzirc_launchers c3_sim_facade.launch
roslaunch mbzirc_launchers c3_starter.launch facade_1_id:=1 (and press ENTER to start)
```  

To run the simulation of fires fires extinction:
```
roslaunch mbzirc_launchers c3_sim_ground.launch
roslaunch mbzirc_launchers c3_starter.launch ground_id:=1 (and press ENTER to start)
```  
