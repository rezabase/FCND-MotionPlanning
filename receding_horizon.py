from enum import Enum
import numpy as np
import math
from queue import PriorityQueue



def reverse_alt_axis(position):
    pos = np.array([position[0], position[1], abs(position[2])])  #Making sure altitude is always +. Reversing the coordinate axis if needed. 
    return pos



###############################
# Receding Horizon 3D Grid
###############################

class HorisonMap3D:
    '''
        self.voxel_size 
        self.halfsize
        self.robot_world_position 
        self.map3d_world_mins = np.array([north_min, east_min, alt_min])
        self.map3d_world_maxs = np.array([north_max, east_max, alt_max])
        
        self.colliders = np.array() #list of colliders that can be loaded from file or added to it manually. this is an np.array
        self.colliders_world_mins = np.array([north_min, east_min, alt_min])
        self.colliders_world_maxs = np.array([north_max, east_max, alt_max])
    '''
    
    def __init__(self, size, init_position, voxel_size=5):
        self.voxel_size = voxel_size
        self.halfsize = np.array(size) * 0.5 #NOTE: *0.5 to save half of the total required size from east to west
        self.update_position_data(init_position)
        self.colliders = np.array([]) #list of colliders that can be loaded from file or added to it manually. this is an np.array
        

    def update_position_data(self, position):
        self.robot_world_position = reverse_alt_axis(position) #current position values
        
        self.map3d_world_mins = (self.robot_world_position - self.halfsize).astype(int)
        self.map3d_world_mins[2] = max(self.map3d_world_mins[2], 0) #limits the altitude to minimum 0, which is ground level.
        
        self.map3d_world_maxs = (self.robot_world_position + self.halfsize).astype(int) 
        
    
    def map3dgrid_from_worldpos(self, world_position):
        world_position = reverse_alt_axis(world_position)
        print("worldposition: " + str(world_position))
        print("self.map3d_world_mins: " + str(self.map3d_world_mins))
        return (world_position - self.map3d_world_mins).astype(int) // self.voxel_size
    
    def worldpos_from_map3dgrid(self, map3d_position):
        map3d_position = np.array(map3d_position)
        return map3d_position * self.voxel_size + self.map3d_world_mins
    
    
    
    def map3dgrid_from_colliderpos(self, collider_position):
        return int(collider_position + self.colliders_world_mins - self.map3d_world_mins) // self.voxel_size
    
    def colliderpos_from_map3dgrid(self, map3d_position):
        return map3d_position * self.voxel_size + self.map3d_world_mins - self.colliders_world_mins
    
    
    
    def append_colliders(self, new_colliders):
        np.append(self.colliders, new_colliders)
        
    def replace_colliders(self, new_colliders):
        self.colliders = new_colliders
        
    def process_colliders_data(self):
        data = self.colliders
        north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
        north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
        east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
        east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))
        alt_min = max(np.ceil(np.amax(data[:, 2] - data[:, 5])), 0) #min can be 0 whihc is ground level
        alt_max = max(np.ceil(np.amax(data[:, 2] + data[:, 5])), 0)
        
        self.colliders_world_mins = np.array([north_min, east_min, alt_min]).astype(int)
        self.colliders_world_maxs = np.array([north_max, east_max, alt_max]).astype(int)
        
    #Give it a world position and it returns a collider grid position.     
    def colliderpos_from_worldpos(self, world_position): 
        world_position = reverse_alt_axis(world_position)
        return int(world_position - self.colliders_world_mins)
    
    def worldpos_from_colliderpos(self, collider_position): 
        collider_position = np.array(collider_position)
        return collider_position + self.colliders_world_mins
    
    
    #Returns true if any of the voxtels have obstacles. 
    def has_obstacle(self):
        return (True in self.voxmap) #if any values in voxmap is true (meaning there is an obstacle), return true
    

    
    def create_voxmap(self, data, new_local_position):
        self.update_position_data(new_local_position) #update drones new local position
        voxmap_size = np.ceil((self.map3d_world_maxs - self.map3d_world_mins) / self.voxel_size).astype(int)
        print("\nvoxtel size: {0}\n".format((voxmap_size[0], voxmap_size[1], voxmap_size[2])))
        
        # Create an empty grid
        voxmap = np.zeros((voxmap_size[0], voxmap_size[1], voxmap_size[2]), dtype=np.bool)

        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]
            obstacle = [
                int(north - d_north   - self.map3d_world_mins[0]) // self.voxel_size,
                int(north + d_north   - self.map3d_world_mins[0]) // self.voxel_size,
                
                int(east - d_east     - self.map3d_world_mins[1]) // self.voxel_size,
                int(east + d_east     - self.map3d_world_mins[1]) // self.voxel_size,
                
                int(alt - d_alt   ) // self.voxel_size, #lowest possible value can be 0. if its -nr, it means its under the ground and we dont want that. 
                int(alt + d_alt   ) // self.voxel_size, 
            ]
            
            #NOTE: I dont need to filter the data range to remove ranges that are outside my voxmap size. if they are out, they are just discarted by np.array wtith the line below, automatically. 
            voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], obstacle[4]:obstacle[5]] = True #one means there is an obstacle
            #print("voxmap[" + str(obstacle[0]) + ":" + str(obstacle[1]) + ", " + str(obstacle[2]) + ":" + str(obstacle[3]) + str(obstacle[4]) + ":" + str(obstacle[5]) +"])")
            
        self.voxmap = voxmap
        return voxmap








###############################
# Potential Field Calculations
###############################


def attraction(position, goal, alpha):
    #implement attraction force
    return alpha * (np.array(position) - np.array(goal))

def repulsion(position, obstacle, beta, q_max):
    #implement replusion force
    position = np.array(position)
    obstacle = np.array(obstacle)
    return beta * ((1 / q_max) - (1 / np.linalg.norm(position - obstacle))) * (1 / np.linalg.norm(position - obstacle)**2)


def potential_field(grid, goal, alpha, beta, q_max):
    x = []
    y = []
    z = []
    fx = []
    fy = []
    fz = []
    field_costs = {}

    obs_ix, obs_iy, obs_iz = np.where(grid == True)

    for ix in range(grid.shape[0]):
        for iy in range(grid.shape[1]):
            for iz in range(grid.shape[2]):
                
                if grid[ix, iy, iz] == False:
                    # add attraction force
                    force = attraction([ix, iy, iz], goal, alpha)

                    for (ox, oy, oz) in zip(obs_ix, obs_iy, obs_iz):
                        if np.linalg.norm(np.array([ix, iy, iz]) - np.array([ox, oy, oz])) < q_max:
                            # add replusion force
                            force += repulsion([ix, iy, iz], [ox, oy, oz], beta, q_max)
                    
                    x.append(ix)
                    y.append(iy)
                    z.append(iz)
                    fx.append(force[0])
                    fy.append(force[1])
                    fz.append(force[2])

                    field_costs[(ix, iy, iz)] =  force

    print("DONE")
    return field_costs





###############################
# VOXMAP 3D Grid A* calc
###############################


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)
    
    #REZA
    UP = (0, 0, 1, 0.5)
    DOWN = (0, 0, -1, 0.5)

    #Diagonal Movements
    NE = (-1, 1,0, math.sqrt(2))
    SE = ( 1, 1,0, math.sqrt(2))
    SW = ( 1,-1,0, math.sqrt(2))
    NW = (-1,-1,0, math.sqrt(2))

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1],  self.value[2])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m, u = grid.shape[0] - 1, grid.shape[1] - 1, grid.shape[2] - 1
    x, y, z = current_node

    # check if the node is off the grid or
    # it's an obstacle

    #print ((x, y, z))
    
    if x - 1 < 0 or grid[x - 1, y, z] == True:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y, z] == True:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1, z] == True:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1, z] == True:
        valid_actions.remove(Action.EAST)

        
    if z - 1 < 0 or grid[x, y, z - 1] == True:
        valid_actions.remove(Action.DOWN)
    if z + 1 > u or grid[x, y, z + 1] == True:
        valid_actions.remove(Action.UP)
        
        
    # Diagonal movements
    if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1, z] == True:
        valid_actions.remove(Action.NW)
    if x - 1 < 0 or y + 1 > m or grid[x-1, y+1, z] == True:
        valid_actions.remove(Action.NE)
    if x + 1 > n or y - 1 < 0 or grid[x+1, y-1, z] == True:
        valid_actions.remove(Action.SW)
    if x + 1 > n or y + 1 > m or grid[x+1, y+1, z] == True:
        valid_actions.remove(Action.SE)

    #print (valid_actions)
    return valid_actions


#original one
#def heuristic(position, goal_position):
#    return np.linalg.norm(np.array(position) - np.array(goal_position))


def heuristic_voxmap(grid_node_position, fx, fy, fz):
    return fx[grid_node_position[0]] + fy[grid_node_position[1]] + fz[grid_node_position[2]] #REZA
    


def a_star_voxmap(grid, start_grid_position, goal_world_position, field_costs_dic):

    start_grid_position = tuple(start_grid_position)
    goal_world_position = reverse_alt_axis(goal_world_position)
    ###start_world_position = reverse_alt_axis(start_world_position)

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start_grid_position))
    visited = set(start_grid_position)

    branch = {}
    found = False
    
    
    highest_branch_cost = 0
    branch_key_with_highest_cost = start_grid_position
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start_grid_position:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        for action in valid_actions(grid, current_node):
            # get the tuple representation
            da = action.delta
            next_node = (current_node[0] + da[0], current_node[1] + da[1], current_node[2] + da[2])
            


            next_node_field_cost = field_costs_dic[(next_node[0], next_node[1], next_node[2])] * -1 #Note: The Field Cost calculation has the costs on oposite direction. I am multiplying with -1 to reverse it. 
            #print("\nnext_node_field_cost: " + str(next_node_field_cost))
            #print("da: " + str(da))
            #print("node fieldcost: " + str(next_node_field_cost*da))
            next_node_cost = da[0]*next_node_field_cost[0] + da[1]*next_node_field_cost[1] + da[2]*next_node_field_cost[2]
            #print("node fieldcost total: " + str(next_node_cost))
            
            branch_cost = current_cost + action.cost*5 + next_node_cost
            #print("fieldcost branch_cost: " + str(branch_cost))
            queue_cost = branch_cost #+ heuristic_voxmap(start_world_position, goal_world_position)
                 
            if next_node not in visited:                
                visited.add(next_node)               
                branch[next_node] = (branch_cost, current_node, action)
                queue.put((queue_cost, next_node))
                
                if branch_cost > highest_branch_cost:
                    highest_branch_cost = branch_cost
                    branch_key_with_highest_cost = next_node
 
    print("\nBranch highest cost: {0}\n".format(branch[branch_key_with_highest_cost]))
    
    n =  branch_key_with_highest_cost #Assuming that the one with highest cost is closest to the goal (and its not blocked)
    path_cost = branch[n][0]
    path.append(n)
    while branch[n][1] != start_grid_position:
        path.append(branch[n][1])
        n = branch[n][1]
    path.append(branch[n][1])
        
    return path[::-1], path_cost