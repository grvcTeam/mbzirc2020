# catkin_mbzirc2020

Catkin repository to store ROS packages in the MBZIRC2020 architecture

NOTE: this is the parent src folder in a typical catkin structure

-------------------

External dependencies:

- shapely python package: pip install shapely
- pydispatch python package: sudo apt install python-pydispatch


-------------------

Run demo instructions:

- Challenge 1 [UNCOMPLETED]:

	roslaunch agent_nodes c1.launch

	rosrun agent_nodes cu_agent_c2.py
	rosservice call /central_unit/task/search_and_build ...

- Challenge 2:

	roslaunch agent_nodes c2.launch

	#just if UGV wants to be added ###

	roslaunch agent_nodes ugv_gazebo_noagent.launch
	roslaunch agent_nodes ugv_agent.launch

	##################################

	rosrun agent_nodes cu_agent_c2.py
	rosservice call /central_unit/task/search_and_catch ...
