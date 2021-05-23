from enum import Enum
from queue import PriorityQueue
import numpy as np
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham
import numpy.linalg as LA

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid_size = (north_size, east_size)
    grid = np.zeros(grid_size)

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min), grid_size

def create_2_dot_5_grid(data, safety_distance):
    """
    Returns a grid representation of a 2.5D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid_size = (north_size, east_size)
    grid = np.zeros(grid_size)

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [
            int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
            int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
            int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
            int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
        ]
        grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = alt + d_alt

    return grid, int(north_min), int(east_min), grid_size


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    SW = (1, -1, np.sqrt(2))
    NE = (-1, 1, np.sqrt(2))
    NW = (-1, -1, np.sqrt(2))
    SE = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
            valid_actions.remove(Action.SE)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NE)
    
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SW)
    
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NW)
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star_grid(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    i = 0
    while not queue.empty():
        i+=1
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def prune(path, epsilon=5):
    
    def pt(p):
        return np.array([p[0], p[1], 1.])

    def isColinear(p1, p2, p3):
        mat = np.vstack((pt(p1), pt(p2), pt(p3)))
        det = np.linalg.det(mat)
        if np.abs(det) < epsilon:
            return True
        else:
            return False

    pruned_path = path
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 1]
        p3 = pruned_path[i + 2]

        if isColinear(pt(p1), pt(p2), pt(p3)):
            pruned_path.remove(p2)
        else:
            i += 1

    return pruned_path

def can_connect(n1, n2):
    '''
    casts two points as a shapely LineString() object
    tests for collision with a shapely Polygon() object
    returns True if connection is possible, False otherwise
    '''

    l = LineString([n1, n2])
    for p in polygons:
        #if p.crosses(l) and p.height >= min(n1[2], n2[2]):
        if p.crosses(l):
            return False
    return True

def create_graph(nodes, k):
    '''
    defines a networkx graph as g = Graph()
    defines a tree = KDTree(nodes)
    test for connectivity between each node and 
    k of it's nearest neighbors
    if nodes are connectable, add an edge to graph
    Iterate through all candidate nodes!
    '''
    
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if np.array_equal(n1, n2):
                continue
            if can_connect(n1, n2):
                g.add_edge(tuple(n1), tuple(n2), weight=1)
    return g

def a_star_graph(graph, heuristic, start_ne, goal_ne):
    """Modified A* to work with NetworkX graphs."""
    
    start = closest_point(graph, start_ne)
    goal = closest_point(graph, goal_ne)
    if start == goal:
        return [], 0
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            #print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost

def get_home_position():
    """
    Returns home position from colliders.csv
    """
    
    f = open ('colliders.csv')
    first_row = f.readlines()[0].replace('\n','').split(', ')
    f.close()
    lat0 = float(first_row[0].strip('lat0 '))
    lon0 = float(first_row[1].strip('lon0 '))
    return lon0, lat0
        
def create_grid_and_edges(data, drone_altitude, safety_distance, resolution = 1):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min) / resolution)
    east_size = int(np.ceil(east_max - east_min) / resolution)

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip((north - d_north - safety_distance - north_min) / resolution, 0, north_size-1)),
                int(np.clip((north + d_north + safety_distance - north_min) / resolution, 0, north_size-1)),
                int(np.clip((east - d_east - safety_distance - east_min) / resolution, 0, east_size-1)),
                int(np.clip((east + d_east + safety_distance - east_min) / resolution, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

            # add center of obstacles to points list
            points.append([int((north - north_min) / resolution), 
                           int((east - east_min) / resolution)])

    # create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return edges
    #return grid, edges, north_min, np.ceil(north_max - north_min), east_min, np.ceil(east_max - east_min)

def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def random_goal(north_range, east_range, 
                start=(0,0,0), min_dist = 0, max_altitude = 20):
    """
    Returns a random goal, expressed in NED format
    ((north, east, down)
    
    Args:
    north_range: tuple(min,max) all inclusive
    easth_range: tuple(min,max) all inclusive
    down_range: tuple(min,max) all inclusive
    start: start location in NED format
    min_dist: integer representing the minimum distance between start and goal
    """
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='float32', skiprows=2)
    dot_5_grid, north_offset, east_offset, _ = create_2_dot_5_grid(data, 5)
    occupied_location = True
    dist = 0
    while dist < min_dist or occupied_location:
        ne_coord = (np.random.randint(north_range[0], north_range[1]+1),
        np.random.randint(east_range[0], east_range[1]+1))
        dist = np.linalg.norm(np.array(ne_coord) - np.array(start))
        #print ("Goal altitude = {}".format(dot_5_grid[ne_coord[0] - north_offset][ne_coord[1] - east_offset]))
        occupied_location = (dot_5_grid[ne_coord[0] - north_offset][ne_coord[1] - east_offset] > max_altitude)
    ned_coord = (ne_coord[0], ne_coord[1], int(dot_5_grid[ne_coord[0] - north_offset][ne_coord[1] - east_offset]))
    return ned_coord

def create_moving_grid(current_loc, name, grid_size, res):
    """
    Creates a grid with the current location as the center of the grid
    """
    
    grid = {
    "name": name,
    "origin": np.array(current_loc[:2]) - np.array(grid_size) / 2,
    "offset": (0,0),
    "resolution": res,
    "size": grid_size,
    "data": np.zeros(grid_size)   
    }
    return grid

def in_grid_3D(m_coord_pt, grid):
    """
    Returns True if a point with map coordinates m_coord_pt is in grid
    """
    g_coord_pt = map_to_grid_3D(m_coord_pt, grid)
    for i in range(len(m_coord_pt)):
        if not (0<=g_coord_pt[i]<grid["size"][i]):
            return False
    return True

def in_grid(m_coord_pt, grid):
    """
    Returns True if a point with map coordinates m_coord_pt is in grid
    """
    g_coord_pt = map_to_grid(m_coord_pt, grid)
    test = (0<=g_coord_pt[0]<grid["size"][0]) and (0<=g_coord_pt[1]<grid["size"][1])
    return test

def intersect_grid_pts (pt_a, pt_b, grid):
    """
    Finds the point at the intersection of the line formed by 2 points and the grid
    Assumes we move from pt_a to pt_b
    """
    
    # Notation: y = mx + b - linear Equation
    if abs((pt_b[1] - pt_a[1])) > 0.1:
        m = (pt_b[0] - pt_a[0])/(pt_b[1] - pt_a[1])
        
        if m == 0:
            b = pt_a[0]
        else:
            b = (pt_b[1] * pt_a[0] - pt_a[1] * pt_b[0]) / (pt_b[1] - pt_a[1])
            
            if pt_b[0] > pt_a[0]:
                y1 = grid["origin"][0] + \
                grid["size"][0] * grid["resolution"] - \
                grid["resolution"] /2
                x1 = (y1 - b) / m
            else:
                y1 = grid["origin"][0] + \
                grid["resolution"] /2
                x1 = (y1 - b) / m

        if pt_b[1] > pt_a[1]:
            x2 = grid["origin"][1] + \
            grid["size"][1] * grid["resolution"] - \
            grid["resolution"] /2
            y2 = m * x2 + b
        else:
            x2 = grid["origin"][1] + \
            grid["resolution"] /2
            y2 = m * x2 + b
        
        #print ("The 2 possible points are: {}, {}".format([y1, x1], [y2, x2]))
        if in_grid(np.array([y2, x2]), grid):
            inter_pt = np.array([y2, x2])
        else:
            inter_pt = np.array([y1, x1])
    
    else:
        x1 = pt_b[1]
        if pt_b[0] > pt_a[0]:
            y1 = grid["origin"][0] + \
            grid["size"][0] * grid["resolution"] - \
            grid["resolution"] /2
        else:
            y1 = grid["origin"][0] + \
            grid["resolution"] /2
        inter_pt = np.array([y1, x1])
        
    return inter_pt

def map_to_grid(m_coord_pt, grid):
    """
    Converts map coordinates to a grid coordinate
    """
    g_coord = [int((m_coord_pt[0] - grid["origin"][0]) / grid["resolution"]),
                 int((m_coord_pt[1] - grid["origin"][1]) / grid["resolution"])]
    return g_coord

def grid_to_map(g_coord, grid):
    """
    Converts grid coordinates to a map coordinate
    """
    m_coord = [g_coord[0] * grid["resolution"] + grid["origin"][0],
                 g_coord[1] * grid["resolution"] + grid["origin"][1]]
    return m_coord

