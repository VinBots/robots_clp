import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import random_goal, a_star_graph, heuristic, create_grid, prune, get_home_position, create_grid_and_edges, closest_point, create_moving_grid, in_grid, intersect_grid_pts, create_2_dot_5_grid, grid_to_map, map_to_grid, a_star_grid

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import sys
#!{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx
from queue import PriorityQueue
import numpy.linalg as LA

SAFETY_DISTANCE = 7
SAFE_LANDING_DISTANCE = 3

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal = (-122.396591, 37.793405, 5.0), goal_ned = None, receding = True):
        super().__init__(connection)

        self.connection = connection
        self.global_goal = global_goal
        self.goal_ned = goal_ned
        self.receding = receding
        
        self.target_position = np.array([0, 0, 0])
        self.waypoints = []
        self.next_waypoint = []
        self.in_mission = True
        self.check_state = {}
        self.north_offset = 0 #-316
        self.east_offset = 0 # -445
        self.coarse_waypoint = []
        self.deadband = 5
        self.target_altitude = 0
        self.init_graph_res = 10
        self.landing_altitude_min = 0.0
        self.landing_altitude_max = 10.0
        self.graph_attempts = 0
        self.landing_start_time = 0
        self.replanning_timer = 0
        self.replanning_freq = 10
        
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
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < self.deadband:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.landing_start_time == 0:
                self.landing_start_time = time.time()
            t1 = time.time()
            #if self.global_position[2] - self.global_home[2] < 0.1:
            if (t1 - self.landing_start_time) > 10 or \
            (abs(self.local_position[2]) < self.landing_altitude + 0.1):
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.init_routine()                    
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
            elif self.flight_state == States.WAYPOINT:
                if self.receding:
                    self.receding_horizon()

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
        #print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        if len(self.waypoints) == 0:
            self.deadband = 0.5

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
        #print("Sending waypoints to simulator ...")
        if receding:
            data = msgpack.dumps(self.next_waypoint)
        else:
            data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)        

    def load_dot_5_map(self):
        self.data = np.loadtxt('colliders.csv', delimiter=',', dtype='float32', skiprows=2)
        grid, north_offset, east_offset, grid_size = create_2_dot_5_grid(self.data, SAFETY_DISTANCE)
        self.dot_5_grid = grid
        self.north_offset = north_offset
        self.east_offset = east_offset
        self.grid_size = grid_size

    def init_goal(self, goal_geodetic = None, goal_ned = None, min_dist = 100):
        """
        Returns a random goal, expressed in NEU format
        ((north, east, up)
        """
        
        if goal_geodetic:
            new_goal_ned = global_to_local(goal_geodetic, self.global_home)
            goal_ned = [int(new_goal_ned[0]),
                       int(new_goal_ned[1]),
                       -int(new_goal_ned[2])]
            print ("GEODETIC GOAL {} to NED {}".format(goal_geodetic, goal_ned))

        if goal_ned == None:
            max_goal_search_trials = 10
            goal_valid = False
            goal_search_trials = 0
            while not goal_valid or goal_search_trials < max_goal_search_trials:                
                goal_search_trials+=1
                goal_ne = self.random_goal()
                goal_valid = self.check_valid_goal(self.init_start_ned, goal_ne, min_dist)
                goal_ned = (goal_ne[0],
                            goal_ne[1],
                            self.ground_control(goal_ne))
                
        else:
            goal_ned = self.adjust_altitude(goal_ned)
            goal_valid = self.check_valid_goal(self.init_start_ned, goal_ned, min_dist)
        
        return goal_ned, goal_valid
    
    def adjust_altitude(self, ned_coord):
        """
        Adjusts the right altitude for a landing point
        """
        return (ned_coord[0], ned_coord[1], self.ground_control(ned_coord))
    
    
    def check_valid_goal (self, start, goal, min_dist):
        """
        Checks the minimum distance between start and goal
        Checks if the landing altitude of the goal is within an acceptable range
        Checks if the landing zone is safe
        """
        
        tests = np.array([self.check_min_distance (start, goal, min_dist),
                 self.check_landing_altitude (goal),
                 self.landing_control(goal)])
        #print ("Tests results: {}".format(tests))
        
        return tests.all()
                    
    def check_landing_altitude (self, coord_ne):
        """
        Checks if a given landing altitude is within a given range defined by hyperparamers
        self.landing_altitude_min
        self.landing_altitude_max
        """
        alt = self.ground_control (coord_ne)
        return (self.landing_altitude_min <= alt <= self.landing_altitude_max)
                        
    def check_min_distance (self, coord_a, coord_b, min_dist):
        """
        Checks if 2 points are separated by a minimum distance "as-the-crow-flies"
        """
        coord_a = np.array(coord_a)[0:2]
        coord_b = np.array(coord_b)[0:2]
        
        dist = np.linalg.norm(np.array(coord_a) - np.array(coord_b))
        return (dist > min_dist)
    
    def random_goal(self):
        """
        Returns a random goal, expressed in NEU format
        ((north, east)
        """
        north = np.random.choice(np.arange(self.north_offset, self.grid_size[0] + self.north_offset - 2), size=1, replace=True)
        east = np.random.choice(np.arange(self.east_offset, self.grid_size[1] + self.east_offset - 2), size=1, replace=True)       
        return (north[0], east[0])
    
    def ground_control(self, ne_coord):
        """
        Returns the altitude of the ground for a given coordinate
        """
        grid_coord = [ne_coord[0] - self.north_offset, ne_coord[1] - self.east_offset]
        return self.landing_grid[grid_coord[0]][grid_coord[1]]
    
    
    def landing_control(self, ne_coord):
        """
        Precisely controls if we can land in a given coordinate - square of safety_distance required
        Returns the landing_altitude
        """

        alt0 = self.ground_control(ne_coord)
        for n in range(-SAFE_LANDING_DISTANCE, SAFE_LANDING_DISTANCE+1):
            for e in range(-SAFE_LANDING_DISTANCE, SAFE_LANDING_DISTANCE+1):
                alt = self.ground_control((ne_coord[0] + n,
                                          ne_coord[1] + e))
                if abs(alt - alt0) > 0.1:
                    return False
        return True
    
    def build_init_graph(self):

        """
        Put in memory the graph of the obstacles map.
        """
        print("Building the graph ...")
        t0 = time.time()
        edges = create_grid_and_edges(self.data, self.target_altitude, SAFETY_DISTANCE, self.init_graph_res)
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)
        print ("Graph creation process took {} seconds".format(int(time.time() - t0)))
        return G

    def build_landing_grid(self):
        
        landing_grid = np.zeros(self.grid_size)
        north_size = self.grid_size[0]
        east_size = self.grid_size[1]

        for i in range(self.data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = self.data[i, :]
            obstacle = [
                int(np.clip(north - d_north - self.north_offset, 0, north_size-1)),
                int(np.clip(north + d_north - self.north_offset, 0, north_size-1)),
                int(np.clip(east - d_east - self.east_offset, 0, east_size-1)),
                int(np.clip(east + d_east - self.east_offset, 0, east_size-1)),
            ]

            landing_grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = alt + d_alt

        return landing_grid
    
    def plan_path (self):
        #self.flight_state = States.PLANNING
        pass
    
    def init_start(self):
        
        lon0, lat0 = get_home_position()
        self.set_home_position(lon0, lat0, 0)
        local_position = global_to_local(self.global_position, self.global_home)
        self.init_start_ned = self.local_position
    
    def plan_initial_path(self):
        
        start_node = np.array((-self.north_offset + self.init_start_ned[0], 
                               -self.east_offset + self.init_start_ned[1])) / self.init_graph_res
        goal_node = np.array((-self.north_offset + self.goal_ned[0],
                              -self.east_offset + self.goal_ned[1])) / self.init_graph_res
        
        path, cost = a_star_graph(self.init_graph, heuristic, start_node, goal_node)

        waypoints_list = [[int(p[0] * self.init_graph_res + self.north_offset),
                           int(p[1] * self.init_graph_res + self.east_offset),
                           int(self.target_altitude), 0] for p in path]
        
        goal_wp = [int(self.goal_ned[0]), int(self.goal_ned[1]), 
                   int(self.target_altitude), 0]
        
        waypoints_list.append(goal_wp)
        
        if len(waypoints_list) > 1:
            self.flight_state = States.PLANNING
            self.coarse_waypoints = waypoints_list
            self.waypoints = waypoints_list # initial waypoints
            self.send_waypoints()
        else:
            self.graph_attempts += 1
            print ("A safe path could not be found... Attempts {} / 4 ".format(self.graph_attempts))
            if self.graph_attempts < 4:
                self.init_graph_res = int(self.init_graph_res / 2)
                print ("Try with a higher resolution graph... {} / 1".format(self.init_graph_res))
                self.init_graph = self.build_init_graph()
            else:
                print ("Goal location not valid. Landing Zone not safe - please change the goal location and restart")
                self.disarming_transition()

    def init_routine(self):
        self.load_dot_5_map()
        print ("Load the map...")
        self.landing_grid = self.build_landing_grid()  
        print ("Load the landing grid")
        self.init_start()
        print ("Startsee initi initializaddtion")

        self.goal_ned, valid_goal = self.init_goal(goal_geodetic = self.global_goal, 
                                                   goal_ned = self.goal_ned)        
        if valid_goal:
            print ("Goal selected with a safe landing zone: GOAL: {}".format(self.goal_ned))
            self.target_altitude = int(self.goal_ned[2]) + SAFETY_DISTANCE
            self.landing_altitude = self.goal_ned[2]
            print ("Build a graph...")
            self.init_graph = self.build_init_graph()
            print ("Checking the path...")
            self.plan_initial_path()
            self.replanning_timer = time.time()
            
        else:
            print ("Goal location not valid - please change the goal location and restart")
            self.disarming_transition()
            
    def start(self):
        self.start_log("Logs", "NavLog.txt")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass
        self.stop_log()

    def find_wp_outside_grid(self, current_loc, my_moving_grid):
        """
        Returns the first waypoint that is outside the grid
        """
        for wp in self.waypoints:
            if not in_grid(wp, my_moving_grid):
                return wp
        
    def find_wp_inside_grid(self, current_loc, my_moving_grid):
        """
        Returns the waypoint that is inside the grid and the furthese in the list of waypoints
        If not found, returns the current_location that is in the middle of the grid
        """
        in_wp = current_loc
        for wp in self.waypoints:
            if in_grid(wp, my_moving_grid):
                in_wp = wp
            else:
                break
        return in_wp
    
    def find_goal_in_grid (self, in_wp, out_wp, my_moving_grid):
        """
        Returns the goal inside the grid. Here it is the intersection between
        the line L and the moving grid
        the line L is formed by 2 points: the furthest waypoint inside the grid 
        and the closest waypoint outside the grid
        """
        
        return intersect_grid_pts(in_wp, out_wp, my_moving_grid)
    
    def cut_old_waypoints(self, out_wp):
        """
        Removes all the waypoints until the waypoint outside the grid
        """
        
        out_wp_index = self.waypoints.index(out_wp)
        return self.waypoints[out_wp_index:]
    
    def define_new_waypoints_inside_grid(self, horizon_goal, my_moving_grid):
        """
        Finds the best path between the center of the moving grid and the horizon goal
        Using A* in a grid
        """
        grid_horizon_goal = tuple(map_to_grid(horizon_goal, my_moving_grid))
        
        my_moving_grid["data"] = self.fill_with_obstacles (my_moving_grid)
        
        current_loc_in_grid = (int(my_moving_grid["size"][0] / 2),
                              int(my_moving_grid["size"][1] / 2))
        
        path, _ = a_star_grid (my_moving_grid["data"], heuristic, current_loc_in_grid, grid_horizon_goal)
        path = prune(path)[1:]
        
        waypoints_list = [[int(grid_to_map(p, my_moving_grid)[0]),
                          int(grid_to_map(p, my_moving_grid)[1]),
                          self.target_altitude, 
                          0] for p in path]
        return waypoints_list
        """        
        # Alternative: cutting corners! 
        
        return [int(horizon_goal[0]),
                int(horizon_goal[1]),
                self.target_altitude,
                0]
                
        """
    
    def fill_with_obstacles (self, my_moving_grid):
        """
        Returns a grid with value = 1 for grids with obstacles
        """
        new_grid_data = my_moving_grid["data"]
        for row in range(my_moving_grid["size"][0]):
            for col in range(my_moving_grid["size"][1]):
                m_coord = grid_to_map((row, col), my_moving_grid)
                dot_5_grid_coord = [int(m_coord[0] - self.north_offset),
                                    int(m_coord[1] - self.east_offset)]
                if self.dot_5_grid[dot_5_grid_coord[0]][dot_5_grid_coord[1]] + SAFETY_DISTANCE - 1 > self.target_altitude:
                    new_grid_data[row][col] = 1
        return new_grid_data
    
    def receding_horizon(self):
        
        
        if (time.time() - self.replanning_timer) > self.replanning_freq:
            print ("Replanning process...")
            grid_size = (31, 31)
            res = 1

            current_loc = (int(self.local_position[0]),
                              int(self.local_position[1])
                          )
            my_moving_grid = create_moving_grid(current_loc, "my_moving_grid", grid_size, res)

            out_wp = self.find_wp_outside_grid(current_loc, my_moving_grid)
            in_wp = self.find_wp_inside_grid(current_loc, my_moving_grid)

            if out_wp:
                horizon_goal = self.find_goal_in_grid(in_wp, out_wp, my_moving_grid)        
                self.waypoints = self.cut_old_waypoints (out_wp)
                self.waypoints = self.define_new_waypoints_inside_grid(horizon_goal, my_moving_grid) + self.waypoints
                self.next_waypoint = [self.waypoints[0]]
                self.send_waypoints()
            else:
                goal_wp = [int(self.goal_ned[0]), int(self.goal_ned[1]), 
                           int(self.target_altitude), 0]
                self.waypoints = [goal_wp]
                
            self.replanning_timer = time.time()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60000)
    
    # list of coordinates for testing purpose
    # altitude set as a positive value
    
    ned_coord_list = [(296, -81, 3), 
                      (0, 0, 0), 
                      (100, 100, 0), 
                      (407, -36, 0), 
                      (95, -276, 0), 
                      (-63, -129, 0), 
                      (248, 79, 0), 
                      (183, -178, 0)]

    geodetic_coord_list = [(-122.39834830783556, 37.79515246607782, 3.0),
                           (-122.39744999999897, 37.79248000012615, 0.0),
                           (-122.39630693595093, 37.79337545780131, 0.0),
                           (-122.39782909011151, 37.79615026622789, 0.0),
                           (-122.4005777005116, 37.79335219860672, 0.0),
                           (-122.39891969293332, 37.79191968249794, 0.0),
                           (-122.39653459236686, 37.794710561515735, 0.0),
                           (-122.39945825085853, 37.794139649856525, 0.0)]

    #######################################################################
    #########################CONFIGURATIONS################################
    
    ######## Set a geodetic coordinate ########
    
    #geodetic_coord = None
    geodetic_coord = geodetic_coord_list[0]
    
    # if geodetic coordinate is set to None, you can set a NEU coordinate
    # if ned_coord is set to None, the goal will be chosen at random
    
    ######## Set a NED coordinate ########
    
    ned_coord = None
    #ned_coord = ned_coord_list[4]
    
    ######## RECEDING HORIZON ########
    
    # set receding at True to use a receding horizon with A* grid search
    # WARNING: it slows down the simulator quite a lot
    receding = False
    
    drone = MotionPlanning(conn, 
                           global_goal = geodetic_coord, 
                           goal_ned = ned_coord, 
                           receding = receding)
    time.sleep(1)
    drone.start()
