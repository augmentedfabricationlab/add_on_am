from compas.datastructures import Mesh, Network
from compas.geometry import Point, KDTree, Plane, Frame, Vector, cross_vectors
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
        new_positions = []
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
                    new_positions.append(index)
                    index += 1
                # different x than pos
                if i!=0 and envelope.r_in < abs(sin_r-y_l) < envelope.r_out:
                    attr_dict = {
                        'x':x_r, 'y':y_l, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    new_positions.append(index)
                    index += 1
                # different j than pos
                if j!=0 and envelope.r_in < abs(sin_l-y_r) < envelope.r_out:
                    attr_dict = {
                        'x':x_l, 'y':y_r, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    new_positions.append(index)
                    index += 1
                # different y and x than pos
                if i!=0 and j!=0 and envelope.r_in < abs(sin_r-y_r) < envelope.r_out:
                    attr_dict = {
                        'x':x_r, 'y':y_r, 'z':0,
                        "ri": 0,
                        "vertices": [],
                        }
                    self.network.add_node(key=index, attr_dict=attr_dict)
                    new_positions.append(index)
                    index += 1
        return new_positions
    

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
                    pos_list[node] = [positions[0]]
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
    def poses(self, network, pos):
        # if pos is on other side flip normals
        flip = False
        if pos.Y < network.node_attribute(network.path[0], "y"):
            flip = True

        frames = []
        for node in network.path:
            frames.append(self.set_node_frame(network, node, flip))        
        return frames
    
    def set_node_frame(self, network, node, flip):
        norm = Vector.from_data(network.node_attributes(key=node, names=['vx','vy','vz']))
        if flip:
            norm = norm.inverted()
        v1 = cross_vectors(norm, Vector.Zaxis()) # Zaxis
        v2 = cross_vectors(norm,v1)
        frame = Frame(Point.from_data(network.node_coordinates(node)), v1, v2)
        network.node_attribute(key=node, name='frame', value=frame)
        return frame


class GrowthPositioning(Map2d_optimized):  
    def segment(self, wall_network, envelope, max_depth=100, reachability_map=None, dynamic=False):
        self.wall_network = wall_network.copy()
        self.nodes_dict = {}
        self.reachable_nodes = []

        start = self.find_pos(Point(0, 0, self.wall.height/2), 3)
        
        self.repositioning = [start]
        self.growth(start, wall_network, envelope, max_depth=max_depth, reachability_map=reachability_map)

        k = 0
        while len(self.reachable_nodes) < self.wall_network.number_of_nodes() and k < 10:
 
            start = self.find_pos(Point(0, 0, self.wall.height/2), 4, dynamic)
            self.repositioning.append(start)
            self.growth(start, wall_network, envelope, max_depth=max_depth, reachability_map=reachability_map)
            k += 1
            print(k)

        self.wall_network = wall_network.copy()
        return self.nodes_dict, self.reachable_nodes, self.repositioning


    def growth(self, current, wall_network, envelope, max_depth=100, reachability_map=None):        
        positions = list(self.network.nodes())
        self.refinement = 0
        current_positions = []
        for pos in positions:
            envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
            inside = envelope.point_inside(*self.wall_network.node_coordinates(current))
            if inside:
                current_positions.append(pos)
        # print("current position: " + str(current) + " " + str(current_positions))
        self.reachable_nodes.append(current)
        self.nodes_dict[current] = current_positions
        self.recursive_growth([current], current_positions, envelope, max_depth=max_depth)
    
    
    def find_pos(self, point, neighbors=4, dynamic=False):
        # get all nodes with 3 neighbors
        candidates = list(self.wall_network.nodes_where({'number_of_neighbors': neighbors}))
        # find point closest to given point
        candidates = [candidate for candidate in candidates if candidate not in self.reachable_nodes]
        if candidates == []:
            candidates = list(self.wall_network.nodes())
            candidates = [candidate for candidate in candidates if candidate not in self.reachable_nodes]
        if dynamic:
            # get farthest point on x between height/2 +- 0.1
            candidates_line = [candidate for candidate in candidates if self.wall_network.node_attribute(key=candidate, name="z") < self.wall.height/2 + 0.1 and self.wall_network.node_attribute(key=candidate, name="z") > self.wall.height/2 - 0.1]
            if candidates_line != []:
                x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in candidates_line]
                k = x_vals.index(min(x_vals))
                return candidates_line[k]
        
        print("candidates: ", len(candidates), len(self.reachable_nodes), self.wall_network.number_of_nodes())
        kd = KDTree([Point(*self.wall_network.node_coordinates(i)) for i in candidates])
        p, k, dist = kd.nearest_neighbor(point)

        return candidates[k]


    def recursive_growth(self, nodes, last_positions, envelope, max_depth=100, depth=0): 
        depth += 1
        next_nodes = []
        for current in nodes:   
            neighbors = self.wall_network.node_attribute(key=current, name="neighbors")
            neighbors = [neighbor for neighbor in neighbors if neighbor not in self.reachable_nodes]
            if neighbors == []:
                continue
            # sort neighbors by x value
            x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in neighbors]
            neighbors = [neighbor for _, neighbor in sorted(zip(x_vals, neighbors))]
            for neighbor in neighbors:
                neighbor_positions = []
                # check if neighbor is already in reachable nodes
                if neighbor in self.reachable_nodes or neighbor in nodes:
                    continue
                for pos in last_positions:
                    envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
                    inside = envelope.point_inside(*self.wall_network.node_coordinates(neighbor))
                    if inside:
                        neighbor_positions.append(pos)
                if neighbor_positions == [] and self.refinement < 4:
                    neighbor_positions = self.optimize_pos(envelope, last_positions[0])
                if neighbor_positions != []:
                    next_nodes.append(neighbor)
                    self.reachable_nodes.append(neighbor)
                    self.nodes_dict[neighbor] = neighbor_positions
                    last_positions = neighbor_positions
        


        if depth < max_depth and next_nodes != []:
            self.recursive_growth(next_nodes, last_positions, envelope, max_depth=max_depth, depth=depth)

    # optimizing the position from a given position by adding a finder grid of positions around the given position
    def optimize_pos(self, envelope, position):
        if self.refinement > 2:
            return []
        if self.refinement == 0:
            res = self.resolution
        else:
            res = self.resolution/(self.refinement*3)
        self.refinement += 1
        # first optimization step (res = self.resolution / 3 = 0.1333)
        # second optimization step (res = self.resolution / 9 = 0.0444)
        new_pos = self.add_to_network(position, envelope, res)
        new_pos.append(position)
        for node in self.reachable_nodes[self.repositioning[-1]:]:
            last_pos = []
            for pos in new_pos:
                envelope.x, envelope.y = self.network.node_attributes(pos, ["x", "y"])
                inside = envelope.point_inside(*self.wall_network.node_coordinates(node))
                if inside:
                    last_pos.append(pos)
            new_pos = last_pos
        print("refinement", position, new_pos)

        return new_pos
    
    def plan_path(self, nodes, distance=0.05):
        # list of nodes with z below distance and in nodes
        lowest_nodes = [node for node in nodes if self.wall_network.node_attribute(key=node, name="z") < distance]
        # start equals node with min(x)
        x_vals = [self.wall_network.node_attribute(key=k, name="x") for k in nodes]
        current = nodes[x_vals.index(min(x_vals))]
        # loop over all the nodes and build path with horizontal lines from left to right
        path = [current]
        self.wall_network.path = [current]
        for _ in nodes:
            if set(path) == set(nodes):
                print("finished path")
                break
            neighbors = self.wall_network.node_attribute(key=current, name="neighbors")
            # choose neighbor with lowest z that is in nodes, but not in path
            neighbors = [neighbor for neighbor in neighbors if neighbor in nodes and neighbor not in path and self.wall_network.node_attribute(key=neighbor, name="z") - self.wall_network.node_attribute(key=current, name="z") < distance/2]
            if neighbors == []:
                # if no neighbor is in nodes, choose node with lowest z and max or min x
                z_vals = [self.wall_network.node_attribute(key=node, name="z") for node in nodes if node not in path]
                z_threshold = min(z_vals) + distance/2
                next_nodes = [node for node in nodes if node not in path and self.wall_network.node_attribute(key=node, name="z") < z_threshold]
                x_vals = [self.wall_network.node_attribute(key=node, name="x") for node in next_nodes] 
                end_nodes = [next_nodes[x_vals.index(min(x_vals))], next_nodes[x_vals.index(max(x_vals))]]
                # choose node that is closer to current
                delta_x_vals = [abs(self.wall_network.node_attribute(key=node, name="x")-self.wall_network.node_attribute(key=current, name="x")) for node in end_nodes]
                next_n = end_nodes[delta_x_vals.index(min(delta_x_vals))]
            else:
                next_n = neighbors[0]
            self.wall_network.add_edge(current, next_n)
            self.wall_network.path.append(next_n)
            path.append(next_n)
            current = next_n
        return path
            
    