import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, read_home_location, new_global_position, prune_path, find_start_goal
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from skimage.morphology import medial_axis
from skimage.util import invert


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 6.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        #Of some strange reason, msgpack.dumps() didnt like the self.waypoints and it generated Type 0 error. 
        #so to get around the problem, I'm recreating the list of waypoints again. This seems like working. 
        test = []
        for item in self.waypoints:
            test.append([int(item[0]), int(item[1]), int(item[2]), int(item[3]) ])
        #data = msgpack.dumps(self.waypoints)
        data = msgpack.dumps(test)
        self.connection._master.write(data)
        

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = read_home_location()

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        current_global_position = [self._longitude, self._latitude, self._altitude]
        print("\nself.global_position: {0}".format(self.global_position))
        print("current_global_position: {0}".format(current_global_position))

        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)
        print("current_local_position: {0}".format(current_local_position))

        print('\nglobal home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset) #hardcoded to middle of the grid, replacing it with below:

        # TODO: convert start position to current position rather than map center
        grid_start = ( int(np.ceil(current_local_position[0] - north_offset)), int(np.ceil(current_local_position[1] - east_offset)) )
        print("grid_start: {0}".format(grid_start))

        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10) #hardcoded as some location 10 m north and 10 m east of map center as dfault, replacing it with below:

        # TODO: adapt to set goal as latitude / longitude position and convert
        #goal_global_position = new_global_position(self.global_home, 15, 25) #adding 20 x 10 meters to the global_home position. 
        goal_global_position = [-122.401055, 37.795461, 0]

        goal_local_position = global_to_local(goal_global_position, self.global_home)
        grid_goal = (int(goal_local_position[0] - north_offset), int(goal_local_position[1] - east_offset))

        print("goal_local_position: {0}".format(goal_local_position))
        print("grid_goal: {0}".format(grid_goal))
        print("grid.shape: {0}".format(grid.shape))

        if grid_goal[0] > grid.shape[0] or grid_goal[1] > grid.shape[1] or grid_goal[0] < 0 or grid_goal[1] < 0:
            print("\n * NOTE: Destination is outside the grid map. setting it back to start ")
            grid_goal = grid_start
            self.landing_transition()

        #if grid[grid_goal[0]][grid_goal[1]] > 0:   # checking that the destination is not colliding with any obstacle 
        #    print("\n * NOTE: Destination is inside an obstacle. Setting it to start position. ")
        #    grid_goal = grid_start
        #    self.landing_transition()


        # REZA: Adding Skeleton median axis
        skeleton = medial_axis(invert(grid))
        skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
        print('\nLocal Skelton Start and Goal: ', grid_start, grid_goal)


        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Grid Start and Goal: ', grid_start, grid_goal)

        # Choose one of the following A*
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal) #Run A* on the grid
        path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        print ("path: ", path)
        path = prune_path(path)
        print ("revised path: ", path)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
