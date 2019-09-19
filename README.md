# mbzirc2020

Catkin repository to store ROS packages in the MBZIRC2020 architecture

## Dependencies
* [grvc-ual](https://github.com/grvcTeam/grvc-ual)
* [grvc-utils](https://github.com/grvcTeam/grvc-utils)

## Run instructions

### Challenge 1 [UNCOMPLETED]:
```
roslaunch mbzirc_launchers c1.launch
```

For visualization:
```
roslaunch mbzirc_visualization c1.launch
```

---
### Challenge 2
```
roslaunch mbzirc_launchers c2.launch
rosrun agent_nodes cu_agent_c2.py
```  

For visualization:
```
roslaunch mbzirc_visualization c2.launch
```
