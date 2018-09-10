# Flying Car Motion-Planning Assignment
Date: Sept 2018


## Task 1: Explain the starter code

I run motion_planning.y and backyard_flyer_solution.py side by side and found the following differences: 

1. States(Enum) have an additional value called PLANNING. Also the assignment of the values have changed to auto() instead of manual assignement 
2. PackyardFlyer class name has changed to MotionPlanning
3. state_callback() function has an additional PLANNING step after ARMING. Plan_path() is called torun the palnning and to find and prepare the waypoints that the drone should follow. 
There is no calculate_box() anymore becouse this time we will be following the waypoints that were created during the planning phase instead of flying within a box. 
4. arming_transition() doesnt set the hme position anymore becouse this time we will be landing at a destination location.
5. There is another new function called send_waypoints, that is used to send the newly planned waypoint data to the simulator. This is for the simulator to be able to visualise the path. 
6. plan_path() funciton is incomplete and needs the planning code that is part of this assigment. 



