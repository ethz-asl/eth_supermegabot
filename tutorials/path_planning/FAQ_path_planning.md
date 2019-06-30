# FAQ Path Planner
__Q: How does the global planner behave if the goal is inside an obstacle?__
__A:__ If the global goal is inside an obstacle, the planner will just print a message on the terminal to warn the user. No new global path will be generated in this case. The __local planner__ will keep following the "old" global path trying to get as close as possible to the global goal.  
To re-start the planning process, just send a new global goal position to the __global planner__. In this case, you don't have to trigger the local planner again, since it has already been started.

__Q: How can we make the global planner work with multiple waypoints?__
__A:__ There are two ways to make this happen:
1. Create a node that sends the waypoints one by one to the global planner.  
If a goal has been reached or if it is unreacheable, then the next waypoint in the list should be sent to the planner. In this case, you will also need to add a ros publisher to the global planner that sends information about the current goal (i.e. if the goal is unreachable or if it has been reached).  
To trigger the global planner, call a service as done [here](https://github.com/VIS4ROB-lab/smb_path_planner/blob/f470b5bc2bb7f7f9ead94f2fa3dfbd26f6f029d0/smb_planner_rviz/src/planning_panel.cpp#L277).  

2. Modify the global planner to accept a list of waypoints (e.g. via a _csv_ or text file). In this case, there is no need for an additional node. The global planner already knows if the goal is reached or if it is unreachable. In both cases, the next waypoint in the list should be used as new global goal.
