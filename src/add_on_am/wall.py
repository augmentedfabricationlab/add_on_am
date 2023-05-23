import math

from compas.datastructures import Mesh
from compas.datastructures import Network
from compas.geometry import Vector, Point, Translation, KDTree
from collections import OrderedDict
from operator import itemgetter
from reachability_map import ReachabilityMap2D


class Wall(object):
    """ Wall generator

     """

    def __init__(self, width=2.0, height=2.0, elsize=0.025, x_disp=0.0, y_disp = 0.0, rotation=0.0):
        """create a regular quad mesh from height and width and edge length in the xz-plane
        """
        
        self.mesh = Mesh()
        self.window_vertices = []
        
        self.elsize = elsize

        self.x_size = int(width / elsize) + 1
        self.z_size = int(height / elsize) + 1

        self.width = elsize*(self.x_size-1)
        self.height = elsize*(self.z_size-1)
        self.rotation = math.pi / 180 * rotation

        self.up_amp=0
        self.up_freq=2.0
        self.up_phase=0.0
        self.down_amp=0
        self.down_freq=2.0
        self.down_phase=0.0

        # create vertices
        for i in range(self.x_size):
            for j in range(self.z_size):
                x = x_disp + math.cos(self.rotation) * i * self.elsize
                y = y_disp + math.sin(self.rotation) * i * self.elsize
                z = j * self.elsize
                glob_id = i*self.z_size+j
                self.mesh.add_vertex(x = x, y = y, z = z, i=i, j=j, glob_id=glob_id, x_disp=0, z_disp=0)

        # create faces
        for i in range(self.x_size - 1):
            for j in range(self.z_size - 1):
                a = i * self.z_size + j
                b = a + self.z_size
                c = b + 1
                d = a + 1
                self.mesh.add_face([a, b, c, d])

    
    def window(self, a=(0.8, 1.2), b=(1.2, 0.8)):
        for vertex in self.mesh.vertices():
            # get x and z coordinates from vertex
            x_val = self.mesh.vertex_attribute(vertex, name='x')
            y_val = self.mesh.vertex_attribute(vertex, name='y')
            z_val = self.mesh.vertex_attribute(vertex, name='z')
            theta = math.cos(self.rotation)*x_val + math.sin(self.rotation)*y_val

            if a[0] < theta < b[0] and a[1] > z_val > b[1]:
                self.mesh.vertex_attribute(vertex, "window", value=True)
                self.window_vertices.append(vertex)
        
    
    def sin_wave(self, amp, freq, phase, value):
        return(amp * math.sin(2.0 * math.pi * freq * value + phase))

    def undulate(self, up_amp=0.075, up_freq=2.0, up_phase=0.0, down_amp=0.075, down_freq=2.0, down_phase=0.0):
        self.up_amp = up_amp
        self.up_freq = up_freq
        self.up_phase = up_phase
        self.down_amp = down_amp
        self.down_freq = down_freq
        self.down_phase = down_phase
        """undulate the mesh with sin waves
        """
        for vertex in self.mesh.vertices():
            # get x coordinate from vertex
            x_val = self.mesh.vertex_attribute(vertex, name='x')
            z_val = self.mesh.vertex_attribute(vertex, name='z')
            y_val = self.mesh.vertex_attribute(vertex, name='y')
            # z coordinate of the point normalized by the height of the wall
            theta = math.cos(self.rotation)*x_val + math.sin(self.rotation)*y_val
            rel_z=z_val/(self.z_size*self.elsize)
            # calculate y value of the up and down part of the wall at this x
            up_y = self.sin_wave(self.up_amp, self.up_freq, self.up_phase, theta)
            down_y = self.sin_wave(self.down_amp, self.down_freq, self.down_phase, theta)
            # specify a y value as a linear blending of the top and the botom of the wall
            shift = rel_z*up_y + (1.0-rel_z)*down_y
            y_shift = y_val - math.cos(self.rotation) * shift
            x_shift = x_val + math.sin(self.rotation) * shift
            # set y coordinate from vertex
            self.mesh.vertex_attribute(vertex, name='y', value=y_shift)
            self.mesh.vertex_attribute(vertex, name='x', value=x_shift)

    
    def zones(self, n):
        for i, face in enumerate(self.mesh.faces()):
            self.mesh.face_attribute(face, "tension", value=False) # reseting everything
            half = len(n) // 2
            force = (n[:half][i] + n[half:][i]) /2
            # force = n[half:][i]
            print(force)
            if force > 0 or abs(force) < 0.005:
                self.mesh.face_attribute(face, "tension", value=True) # if n positive => tension
        print(len(n), i)


    def numpy_test_function(self, value):
        from compas.rpc import Proxy
        np = Proxy('numpy')
        linalg = Proxy('numpy.linalg')

        print("running numpy test function")

        a = np.array([[1, 2], [3, 5]])
        b = np.array([1, 2*value])
        x = linalg.solve(a, b)

        return x
    

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
        self.length = 0

    def create_network(self, envelope):
        reach = int((envelope.r_out - envelope.r_in) / self.resolution) + 1
        index = 0
        for i in range(self.x_size):
            x = i*self.resolution
            y = -self.wall.sin_wave(self.wall.down_amp, self.wall.down_freq, self.wall.down_phase, x)
            for j in range(reach):
                y_l = y + envelope.r_in + j*self.resolution
                y_r = y - envelope.r_in - j*self.resolution
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
        self.length = index
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
        print("resolution: ", new_res, size)
        index = self.length + 1
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
        self.length = index
    

    # finding the the ideal position out of an envelope of positions from all positions and from a given starting node
    def pos_by_path_limited(self, planner, envelope, current_pos, start_node, dist=None):
        start_pos = self.length + 1
        self.add_to_network(current_pos, envelope, dist)
        positions = self.network.nodes()
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
                if pos >= start_pos:
                    [envelope.x, envelope.y] = self.network.node_attributes(pos, ["x", "y"])
                    [x, y, z] = planner.network.node_attributes(node, ["x", "y", "z"])
                    if envelope.point_inside(x, y, z):
                        reachable_pos.append(pos)
            if len(reachable_pos) == 0:
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                return positions[0], reachable_points
            else:
                positions = reachable_pos
                reachable_points.append(node)
        return positions[-1], reachable_points
    

    # optimizing the position from a given position by adding a finder grid of positions around the given position
    def optimize_pos(self, planner, envelope, ideal_pos, reachable_points, loops=2):
        for i in range(loops):
            if i > 0:
                res = self.resolution/(i*3)
            else:
                res = self.resolution
            # first optimization step (res = self.resolution / 3 = 0.1333)
            # second optimization step (res = self.resolution / 9 = 0.0444)
            improved_pos, improved_points = self.pos_by_path_limited(planner, envelope, ideal_pos, reachable_points[0], res)
            if len(improved_points) > len(reachable_points):
                ideal_pos = improved_pos
                reachable_points = improved_points
        return ideal_pos, reachable_points
    

    # finding the ideal positions to print as much as possible of the continous path
    def pos_by_path(self, planner, envelope, reachability_map):
        reach = {}
        positions = self.network.nodes()
        reachable_points = []
        kd = KDTree(reachability_map.points)
        # run is used to skip the nodes after the optimization
        run = True
        last_node = None
        for node in planner.network.path:
            print("node: ", node)
            if node == last_node:
                run = True
                continue
            if not run:
                continue
            reachable_pos = []
            # looping over the positions from which the previous nodes were reachable and appending them to the next iteration if current node is also reachable
            for pos in positions:
                [envelope.x, envelope.y] = self.network.node_attributes(pos, ["x", "y"])
                [x, y, z] = planner.network.node_attributes(node, ["x", "y", "z"])
                ri = self.reachability(reachability_map, kd, envelope.x, envelope.y, x, y, z)
                if pos == 1:
                    print("dist: ", (envelope.x-x)**2 + (envelope.y-y)**2, "ri: ", ri)
                if envelope.point_inside(x, y, z) and ri > 10.0:
                    reachable_pos.append(pos)
            if len(reachable_pos) == 0:
                # ideal_pos, reachable_points = self.optimize_pos(planner, envelope, positions[0], reachable_points)
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                reach[positions[0]] = reachable_points

                # if node != reachable_points[-1]:
                #     print("last node: ", last_node, reachable_points[-1])
                #     last_node = reachable_points[-1]
                #     run = False

                positions = list(self.network.nodes())
                reachable_points = []
            else:
                positions = reachable_pos
                reachable_points.append(node)
        
        # for some reason dicts cant be ordered as wanted and are always ordered by length of items, no idea who had this ingenius idea
        # last value of list is choosen for last position, as least overlap with previous position???

        # optimization for last one not necessary as there is a whole envelope of points that works
        # ideal_pos, reachable_points = self.optimize_pos(planner, envelope, positions[-1], reachable_points)
        reach[positions[-1]] = reachable_points

        return reach, positions
    
    def reachability(self, reachability_map, kd, pos_x, pos_y, node_x, node_y, node_z):
        # trans = Translation.from_vector(Vector(pos_x, pos_y, 0))
        # cloud = Point.transformed_collection(reachability_map.points, trans)
        # kd = KDTree(reachability_map.points)
        p, k, dist = kd.nearest_neighbor(Point(node_x-pos_x, node_y-pos_y, node_z))
        return reachability_map.spheres[k].ri