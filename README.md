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


#### To summarise, the code does the following: 

1. Connects tot he simulator using MavlinkConnect()
2. Initiates some parameters and registers 3 callback functions that will be called by the flight controller based on the state of the flight. 
3. the state_callback funciotn initiates ARMING, and PLANNING phases. 
4. Plan_path() funciton, loads the 2.5D obstacle list (colliders.csv). it converts it to usable data, calculates the planning path and creates the waypoints that the drovne will follow. 
5. positioning events call the local_position_callback() funciton periodically and it will detect when the drone reaches  intermediate drone destinations until the last waypoint. After reaching the last waypoint, it will initiate the landing.


## Task 2: Implementing Your Path Planning Algorithm

#### 2a) Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())

1. Added a new function called read_home_location() in the planning_utils.py file
2. Added the following code to plan_path():

```python
        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = read_home_location()

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        current_global_position = self.global_position

        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)
```        

#### 2b) Retrieve your current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. Then use the utility function global_to_local() to convert to local position (using self.global_home as well, which you just set)

Added the following: 

```python
        # TODO: retrieve current global position
        current_global_position = [self._longitude, self._latitude, self._altitude]
        print("self.global_position: {0}".format(self.global_position))
        print("current_global_position: {0}".format(current_global_position))

        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)
        print("current_local_position: {0}".format(current_local_position))
```        

#### 2c) In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

Added the following: 

```python
        #grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center
        grid_start = ( int(np.ceil(current_local_pos[0]-north_offset)), int(np.ceil(current_local_pos[1]-east_offset) )
```     

#### 2d) In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

Added the following code to:
1. can take a global longitude, latitude and altitude
2. convert it from global position to local and grid positions. 
3. check in the grid position if it is an abstacle. if it is, warn the use and set the destination to start point to alert the user. 

```python
        custom_goal = [180, 80, self.global_home[2]] #give custom global longitude (north) and latitude (east) and altitude
        goal_local_position = global_to_local(custom_goal, self.global_home)
        goal_grid_position = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset))

        if grid[goal_grid_position[0]][goal_grid_position[1]] > 0:   # checking that the destination is not colliding with any obstacle 
            print("\n Destination is inside an obstacle. Setting it to start position. ")
            grid_goal=grid_start
```   



