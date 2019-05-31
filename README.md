# catkin_mbzirc2020

Catkin repository to store ROS packages in the MBZIRC2020 architecture

NOTE: this is the parent src folder in a typical catkin structure

-------------------

External dependencies:

- shapely python package: pip install shapely
- pydispatch python package: sudo apt install python-pydispatch


-------------------

Run demo instructions:

- Challenge 2:

	roslaunch agent_nodes c2.launch
	rosrun build_wall_simple_scheduler build_wall_simple_scheduler.py
	rosservice call /build_wall "{}"
