import math

from compas.datastructures import Mesh
from compas.datastructures import Network
from compas.geometry import Vector, Point, Translation, KDTree
from collections import OrderedDict
from operator import itemgetter


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

