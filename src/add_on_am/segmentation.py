from compas.datastructures import Mesh, Network
from compas.geometry import Point, KDTree, Plane, Frame
from collections import OrderedDict
from operator import itemgetter
import math


class Map2d:
    def __init__(self, wall, resolution=0.05):
        self.network = Network(name="network")
        self.network.path = []
        self.network.default_node_attributes = {
            'x':0, 'y':0, 'z':0,
            "ri": 0,
            "vertices": [],
        }
        self.resolution = resolution
        self.x_size = int(wall.width / resolution) + 1
        self.wall = wall

    def create_network(self, envelope):
        index = 0
        for i in range(self.x_size):
            x = i*self.resolution
            y_down = -self.wall.sin_wave(self.wall.down_amp, self.wall.down_freq, self.wall.down_phase, x)
            y_up = -self.wall.sin_wave(self.wall.up_amp, self.wall.up_freq, self.wall.up_phase, x)
            dynamic_reach = int((envelope.r_out - envelope.r_in + abs(y_down-y_up)) / self.resolution) + 1
            for j in range(dynamic_reach):
                y_l = y_down + envelope.r_in + j*self.resolution
                y_r = y_down - envelope.r_in - j*self.resolution
                attr_dict = {
                    'x':x, 'y':y_l, 'z':0,
                    "ri": 0,
                    "vertices": [],
                    }
                self.network.add_node(key=index, attr_dict=attr_dict)
                index += 1
                attr_dict = {
                    'x':x, 'y':y_r, 'z':0,
                    "ri": 0,
                    "vertices": [],
                    }
                self.network.add_node(key=index, attr_dict=attr_dict)
                index += 1
        return self.network
    
    def assign_reach(self, envelope):
        for node in self.network.nodes():
            for vertex in self.wall.mesh.vertices():
                [envelope.x, envelope.y, ri, vertices] = self.network.node_attributes(node, ["x", "y", "ri", "vertices"])
                [x, y, z] = self.wall.mesh.vertex_coordinates(vertex)
                if envelope.point_inside(x, y, z):
                    ri += 1
                    vertices.append(vertex)
                    self.network.node_attributes(node, ["ri", "vertices"], [ri, vertices])


    def pos_by_path(self, planner, envelope):
        reach = {}
        positions = self.network.nodes()
        reachable_points = []
        for node in planner.network.path:
            reachable_pos = []
            # looping over the positions from which the previous nodes were reachable and appending them to the next iteration if current node is also reachable
            for pos in positions:
                [envelope.x, envelope.y] = self.network.node_attributes(pos, ["x", "y"])
                [x, y, z] = planner.network.node_attributes(node, ["x", "y", "z"])
                if envelope.point_inside(x, y, z):
                    reachable_pos.append(pos)
            if len(reachable_pos) == 0:
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                reach[positions[0]] = reachable_points
                positions = self.network.nodes()
                reachable_points = []
            else:
                positions = reachable_pos
                reachable_points.append(node)
        
        # for some reason dicts cant be ordered as wanted and are always ordered by length of items, no idea who had this ingenius idea
        # last value of list is choosen for last position, as least overlap with previous position???
        reach[positions[-1]] = reachable_points

        return reach, positions
    

    def divide(self, planner, envelope):
        reach, others = self.pos_by_path(planner, envelope)
        positions = {}
        for pos in reach.keys():
            positions[pos] = []
        
        for node in planner.network.nodes():
            [node_x, node_y] = planner.network.node_attributes(node, ["x", "y"])
            distances = []
            for pos in positions.keys():
                [pos_x, pos_y] = self.network.node_attributes(pos, ["x", "y"])
                distances.append((pos_x - node_x)**2 + (pos_y - node_y)**2)
            closest_i = min(range(len(distances)), key=distances.__getitem__)
            positions[positions.keys()[closest_i]].append(node)
        
        sorted_dict = sorted(positions.items(), key=itemgetter(0))
        positions = OrderedDict(sorted_dict)

        return positions, others


class Map2d_optimized(Map2d):
    # adding a finder grid of positions to the given network of positions at a given position with a given resolution (dist/3)    
    def add_to_network(self, pos, envelope, dist=None):
        [pos_x, pos_y] = self.network.node_attributes(pos, ["x", "y"])
        number_p = 9
        size = int(math.sqrt(number_p))
        if dist == None:
            dist = self.resolution
        new_res = dist/size
        # print("resolution: ", new_res, size)
        index = self.network.number_of_nodes() + 1
        for i in range(size):
            x_l = pos_x + i* new_res
            x_r = pos_x - i* new_res
            sin_l = -self.wall.sin_wave(self.wall.down_amp, self.wall.down_freq, self.wall.down_phase, x_l)
            sin_r = -self.wall.sin_wave(self.wall.down_amp, self.wall.down_freq, self.wall.down_phase, x_r)
            for j in range(size):
                y_l = pos_y + j* new_res
                y_r = pos_y - j* new_res
                # not pos
                if (i!=0 or j!=0) and envelope.r_in < abs(sin_l-y_l) < envelope.r_out :
                    attr_dict = {
                        'x':x_l, 'y':y_l, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    index += 1
                # different x than pos
                if i!=0 and envelope.r_in < abs(sin_r-y_l) < envelope.r_out:
                    attr_dict = {
                        'x':x_r, 'y':y_l, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    index += 1
                # different j than pos
                if j!=0 and envelope.r_in < abs(sin_l-y_r) < envelope.r_out:
                    attr_dict = {
                        'x':x_l, 'y':y_r, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    index += 1
                # different y and x than pos
                if i!=0 and j!=0 and envelope.r_in < abs(sin_r-y_r) < envelope.r_out:
                    attr_dict = {
                        'x':x_r, 'y':y_r, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    index += 1
    

    # finding the the ideal position out of an envelope of positions from all positions and from a given starting node
    def pos_by_path_limited(self, planner, envelope, reachability_map, kd, current_pos, start_node, pos_list, dist=None, ri_calc="linear", ri_threshold=0):
        start_pos = self.network.number_of_nodes()
        self.add_to_network(current_pos, envelope, dist)
        positions = list(self.network.nodes())
        reachable_points = []
        # run is used to skip until the start node for this section of the path
        run = False
        for node in planner.network.path:
            if node == start_node:
                run = True
            if not run:
                continue
            reachable_pos = []
            # looping over the positions from which the previous nodes were reachable and appending them to the next iteration if current node is also reachable
            for pos in positions:
                if pos > start_pos:
                    [envelope.x, envelope.y] = self.network.node_attributes(pos, ["x", "y"])
                    [x, y, z] = planner.network.node_attributes(node, ["x", "y", "z"])
                    if ri_calc == "linear":
                        ri = self.linear_reach(envelope, x, y, z)
                    else:
                        ri = self.reachability(reachability_map, kd, envelope.x, envelope.y, x, y, z)
                    
                    inside = envelope.point_inside(x, y, z)
                    if inside and ri > ri_threshold:
                        reachable_pos.append(pos)
                    elif inside and ri <= ri_threshold:
                        # wrong, checks for wrong node
                        if self.check_pose(node, reachability_map, kd, envelope):
                            reachable_pos.append(pos)

            if len(reachable_pos) == 0:
                if not node in pos_list.keys():
                    pos_list[node] = positions[0]
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                return positions[0], reachable_points, pos_list
            else:
                positions = reachable_pos
                reachable_points.append(node)
                if not node in pos_list.keys():
                    pos_list[node] = positions
        return positions[0], reachable_points, pos_list
    

    # optimizing the position from a given position by adding a finder grid of positions around the given position
    def optimize_pos(self, planner, envelope, reachability_map, kd, ideal_pos, reachable_points, pos_list, loops=3):
        for i in range(loops):
            if i > 0:
                res = self.resolution/(i*3)
            else:
                res = self.resolution
            # first optimization step (res = self.resolution / 3 = 0.1333)
            # second optimization step (res = self.resolution / 9 = 0.0444)
            print(ideal_pos)
            improved_pos, improved_points, improved_pos_list = self.pos_by_path_limited(planner, envelope, reachability_map, kd, ideal_pos, reachable_points[0], pos_list, res)
            print(reachable_points, improved_points)
            print("old positions: " + str(len(pos_list)) + " opt positions: " + str(len(improved_pos_list)))
            if len(improved_points) > len(reachable_points):
                ideal_pos = improved_pos
                reachable_points = improved_points
                pos_list = improved_pos_list
        return ideal_pos, reachable_points, pos_list
    

    # finding the ideal positions to print as much as possible of the continous path
    def pos_by_path(self, planner, envelope, reachability_map=None, ri_calc="linear", ri_threshold=0):
        reach = {}
        positions = list(self.network.nodes())
        positions_list = {"start": positions}
        reachable_points = []
        if reachability_map:
            kd = KDTree(reachability_map.points)
        else:
            kd = None
        # run is used to skip the nodes after the optimization
        run = True
        last_node = 0
        for node in planner.network.path:
            # print(run, node, last_node)
            if planner.network.path.index(node) == planner.network.path.index(last_node) and node > 0:
                run = True
                continue
            elif planner.network.path.index(node) > planner.network.path.index(last_node) and not run:
                print("Node advanced too far")
                run = True
            elif not run:
                continue
            reachable_pos = []
            # looping over the positions from which the previous nodes were reachable and appending them to the next iteration if current node is also reachable
            for pos in positions:
                [envelope.x, envelope.y] = self.network.node_attributes(pos, ["x", "y"])
                [x, y, z] = planner.network.node_attributes(node, ["x", "y", "z"])
                if ri_calc == "linear":
                    ri = self.linear_reach(envelope, x, y, z)
                else:
                    ri = self.reachability(reachability_map, kd, envelope.x, envelope.y, x, y, z)
                
                inside = envelope.point_inside(x, y, z)
                if inside and ri > ri_threshold:
                    reachable_pos.append(pos)
                elif inside and ri <= ri_threshold:
                    # wrong, checks for wrong node
                    if self.check_pose(node, reachability_map, kd, envelope):
                        reachable_pos.append(pos)


            if len(reachable_pos) == 0:
                ideal_pos, reachable_points, positions_list_opt = self.optimize_pos(planner, envelope, reachability_map, kd, positions[0], reachable_points, positions_list)
                print("old: " + str(positions[0]) + " opt: " + str(ideal_pos))
                print("old positions: " + str(len(positions_list)) + " opt positions: " + str(len(positions_list_opt)))
                # ideal_pos = positions[0]
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                reach[ideal_pos] = reachable_points
                positions_list = positions_list_opt
                
                # check if final node is reached
                if planner.network.path.index(reachable_points[-1])+1 == len(planner.network.path):
                    print("last node: ", last_node, reachable_points[-1])
                    last_node = reachable_points[-1]
                    run = False
                    return reach, positions, positions_list
                # check if advancement was made through optimization and jump to that node
                elif node != planner.network.path[planner.network.path.index(reachable_points[-1])+1]:
                    print("last node: ", last_node, reachable_points[-1])
                    last_node = reachable_points[-1]
                    run = False

                positions = list(self.network.nodes())
                reachable_points = []
            else:
                positions = reachable_pos
                reachable_points.append(node)
                positions_list[node] = positions
            
            
        
        # for some reason dicts cant be ordered as wanted and are always ordered by length of items, no idea who had this ingenius idea
        # last value of list is choosen for last position, as least overlap with previous position???

        # optimization for last one not necessary as there is a whole envelope of points that works
        # ideal_pos, reachable_points = self.optimize_pos(planner, envelope, positions[-1], reachable_points)
        # print(positions[-1], reachable_points)
        reach[positions[-1]] = reachable_points
        print("length: " + str(len(positions_list.keys())) + " " + str(len(set(positions_list.keys()))))
        return reach, positions, positions_list
    

    def reachability(self, reachability_map, kd, pos_x, pos_y, node_x, node_y, node_z):
        # trans = Translation.from_vector(Vector(pos_x, pos_y, 0))
        # cloud = Point.transformed_collection(reachability_map.points, trans)
        # kd = KDTree(reachability_map.points)
        if reachability_map:
            p, k, dist = kd.nearest_neighbor(Point(node_x-pos_x, node_y-pos_y, node_z))
            return reachability_map.spheres[k].ri
        else:
            return 100
        

    def linear_reach(self, envelope, node_x, node_y, node_z):
        if node_z > envelope.h_out+envelope.z:
            d = math.sqrt((node_x-envelope.x)**2 + (node_y-envelope.y)**2 + (node_z-envelope.h_out-envelope.z)**2)
        elif node_z < envelope.z:
            d = math.sqrt((node_x-envelope.x)**2 + (node_y-envelope.y)**2 + (node_z-envelope.z)**2)
        else:
            d = math.sqrt((node_x-envelope.x)**2 + (node_y-envelope.y)**2)

        linear_out = 0
        linear_in = 0
        if d < envelope.r_out:
            linear_out = (1 - d/envelope.r_out) * 50
        if d > envelope.r_in and d < 2*envelope.r_in:
            linear_in = (d/envelope.r_in - 1) * 50
        elif d > 2*envelope.r_in:
            linear_in = 50
        return linear_out + linear_in
    

    def check_pose(self, node, reachability_map, kd, envelope):
        normal = self.wall.mesh.vertex_normal(node)
        node_x, node_y, node_z = self.wall.mesh.vertex_coordinates(node)
        p, k, dist = kd.nearest_neighbor(Point(node_x-envelope.x, node_y-envelope.y, node_z))
        poses = reachability_map.spheres[k].poses
        for pose in poses:
            # check if angle between normals is less than 20 degree
            angle = pose.normal.angle(normal)*180/math.pi
            if angle < 45:
                return True
        return False
        
    # returns pose at that node   
    def pose(self, node, planner, flip=False):
        normal = planner.network.node_attributes(node, ['vx', 'vy', 'vz'])
        x, y, z = planner.network.node_coordinates(node)
        return Frame.from_plane(Plane(Point(x, y, z), normal))
    
    # returns poses of all nodes that are poses ordered along the path
    def poses(self, nodes, planner, pos):
        # if pos is on other side flip normals
        flip = False
        if pos.Y < planner.network.node_attribute(nodes[0], "y"):
            flip = True

        frames = []
        for node in nodes:
            frames.append(planner.set_node_frame(node, flip))        
        return frames


class GrowthPositioning(Map2d_optimized):  
    
        
        # # sort by x value and get first half
        # x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in candidates]
        # candidates = [candidate for _, candidate in sorted(zip(x_vals, candidates))][:int(len(candidates)/2)]
        # # sort by z value and return closest to height/2
        # z_vals = [abs(self.wall_network.node_attribute(key=k, name="z")-self.wall.height/2) for k in candidates]
        # return candidates[z_vals.index(min(z_vals))]

        # def segment(self, wall_network, envelope, max_depth=100, reachability_map=None):
        # self.wall_network = wall_network.copy()
        # counter = 0
        # self.reachable_nodes = {}
        # while self.wall_network.number_of_nodes() > 0 and counter < 7:
        #     start = self.find_pos(Point(0, 0, self.wall.height/2), 3)
        #     self.growth(start, wall_network, envelope, max_depth=max_depth, reachability_map=reachability_map)
        #     print("nodes: " + str(self.reachable_nodes.keys()))
        #     for node in self.reachable_nodes.keys():
        #         self.wall_network.delete_node(node)
        #     counter += 1
        #     print("counter: " + str(counter) + " nodes: " + str(self.wall_network.number_of_nodes()))
        # self.wall_network = wall_network.copy()
        # return self.reachable_nodes

    def segment(self, wall_network, envelope, max_depth=100, reachability_map=None):
        self.wall_network = wall_network.copy()
        self.reachable_nodes = {}
        
        start = self.find_pos(Point(0, 0, self.wall.height/2), 3)
        self.growth(start, wall_network, envelope, max_depth=max_depth, reachability_map=reachability_map)

        # for node in self.reachable_nodes.keys():
        #     self.wall_network.delete_node(node)
        k = 0
        while len(self.reachable_nodes.keys()) < self.wall_network.number_of_nodes() and k < 6:
            start = self.find_pos(Point(0, 0, self.wall.height/2), 4)
            self.growth(start, wall_network, envelope, max_depth=max_depth, reachability_map=reachability_map)
            k += 1

        self.wall_network = wall_network.copy()
        return self.reachable_nodes


    def growth(self, current, wall_network, envelope, max_depth=100, reachability_map=None):        
        positions = list(self.network.nodes())
        current_positions = []
        for pos in positions:
            envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
            inside = envelope.point_inside(*self.wall_network.node_coordinates(current))
            if inside:
                current_positions.append(pos)
        # print("current position: " + str(current) + " " + str(current_positions))
        self.reachable_nodes[current] = current_positions
        self.recursive_growth([current], current_positions, envelope, max_depth=max_depth)
    
    
    def find_pos(self, point, neighbors=4):
        # get all nodes with 3 neighbors
        candidates = list(self.wall_network.nodes_where({'number_of_neighbors': neighbors}))
        # find point closest to given point
        candidates = [candidate for candidate in candidates if candidate not in self.reachable_nodes.keys()]
        kd = KDTree([Point(*self.wall_network.node_coordinates(i)) for i in candidates])
        p, k, dist = kd.nearest_neighbor(point)
        return candidates[k]


    def recursive_growth(self, nodes, last_positions, envelope, max_depth=100, depth=0): 
        depth += 1
        next_nodes = []
        for current in nodes:   
            neighbors = self.wall_network.node_attribute(key=current, name="neighbors")
            neighbors = [neighbor for neighbor in neighbors if neighbor not in self.reachable_nodes.keys()]
            if neighbors == []:
                continue
            # sort neighbors by x value
            x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in neighbors]
            neighbors = [neighbor for _, neighbor in sorted(zip(x_vals, neighbors))]
            for neighbor in neighbors:
                neighbor_positions = []
                # check if neighbor is already in reachable nodes
                if neighbor in self.reachable_nodes.keys() or neighbor in nodes:
                    continue
                for pos in last_positions:
                    envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
                    inside = envelope.point_inside(*self.wall_network.node_coordinates(neighbor))
                    if inside:
                        neighbor_positions.append(pos)
                if neighbor_positions != []:
                    next_nodes.append(neighbor)
                    self.reachable_nodes[neighbor] = neighbor_positions
                    last_positions = neighbor_positions

        if depth < max_depth and next_nodes != []:
            # positions = set(self.reachable_nodes[next_nodes[0]])
            # for i, node in enumerate(next_nodes[1:]):
            #     new_positions = positions.intersection(self.reachable_nodes[node])
            #     if len(new_positions) == 0:
            #         positions = positions[0]
            #     elif len(new_positions) == 1:

            self.recursive_growth(next_nodes, last_positions, envelope, max_depth=max_depth, depth=depth)

# def recursive_grow(self, current, envelope, max_depth, depth=0):
#     # get neighbors of current node
#     depth += 1
#     neighbors = self.wall_network.node_attribute(key=current, name="neighbors")
#     # sort neighbors by x value
#     x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in neighbors]
#     neighbors = [neighbor for _, neighbor in sorted(zip(x_vals, neighbors))]
#     for neighbor in neighbors:
#         # check if neighbor is already in reachable nodes
#         if neighbor in self.reachable_nodes.keys():
#             continue
        
#         positions = []
#         # check if neighbor is in envelope
#         for pos in self.reachable_nodes[current]:
#             envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
#             inside = envelope.point_inside(*self.wall_network.node_coordinates(neighbor))
#             if inside:
#                 positions.append(pos)
#         # if neighbor is in envelope add to reachable nodes and grow from there
#         print(depth, max_depth, neighbor)
#         if positions != [] and depth < max_depth:
#             self.reachable_nodes[neighbor] = positions
#             self.recursive_grow(neighbor, envelope, max_depth=max_depth, depth=depth)
#         elif depth >= max_depth:
#             print("max depth reached")
#             return

# def growth(self, wall_network, envelope, max_depth, reachability_map=None):
#     self.wall_network = wall_network
#     positions = list(self.network.nodes())
#     current = self.find_pos(Point(0, 0, self.wall.height/2), 3)
    
#     current_positions = []
#     for pos in positions:
#         envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
#         inside = envelope.point_inside(*self.wall_network.node_coordinates(pos))
#         if inside:
#             current_positions.append(pos)
#     print("current position: " + str(current) + " " + str(current_positions))
#     self.reachable_nodes = {current: current_positions}
#     self.recursive_grow(current, envelope, max_depth=max_depth)
#     return self.reachable_nodes

    