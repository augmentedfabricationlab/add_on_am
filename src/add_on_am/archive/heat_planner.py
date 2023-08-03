import math


from compas.datastructures import Network
from compas.geometry import Vector, Point, KDTree, barycentric_coordinates
from compas.rpc import Proxy


from add_on_am.cart_sphe import magnitude, shift_origin, distance


class HeatPathPlanner():
    def __init__(self):
        self.mesh = None
        self.isolines = Network(name="isolines")
        self.isolines.path = []
        self.isolines.default_node_attributes = {
            'x':0, 'y':0, 'z':0,
        }
        self.fabrication_parameters = {
            'material_flowrate':500.0,          # l/hr
            'thickness_range':[0.01, 0.01],     # m [0.008, 0.022]
            'radius_range':[0.040, 0.075],        # m
            'distance_range':[0.100, 0.300],      # m
            'measured_radii':[0.0625, 0.075],        # [m]
            'measured_distances':[0.150, 0.300],  # [m]
            'measured_thicknesses':[0.012, 0.009], # [m]
        }

    def set_quad_mesh(self, mesh):
        self.mesh = mesh
        return self.mesh

    def set_quad_mesh_from_rhinomesh(self, rhinomesh):
        self.mesh = rhinomesh.to_compas()
        return self.mesh

    def create_quad_mesh_from_surface(self, surface, nu, nv):
        self.mesh = surface.to_compas_mesh()
        return self.mesh
    
    
    def heat_method(self, length, height, crv_length, direction="x"):
        if self.mesh == None:
            raise ValueError
        if self.mesh.is_trimesh():
            raise ValueError
        # identify the start and end vertices (for x: left / right edge) 
        start = []
        end = []
        if direction == "x":
            for v in self.mesh.vertices():
                if self.mesh.vertex_coordinates(v)[0] < 0.04:
                    start = v
                    break
            for v in self.mesh.vertices():
                if length - 0.04 < self.mesh.vertex_coordinates(v)[0]:
                    end = v
        
        # calculate geodesic distances from the start vertex/vertices and from the end vertix/vertices
        ds = Proxy("compas.datastructures")
        distances_start = ds.mesh_geodesic_distances_numpy(self.mesh, start, m=1.0)
        distances_end = ds.mesh_geodesic_distances_numpy(self.mesh, end, m=1.0)

        # not implemented in compas yet
        # distances_start = trimesh_geodistance(self.mesh, start)
        # distances_end = trimesh_geodistance(self.mesh, end)


        # find the geodesic distance between the start and end vertices
        d_max_start = distances_start[end]
        d_max_end = distances_end[start]

        # layer height = 0.05 => total distances / 0.05 = number of layers
        avg_thickness = self.fabrication_parameters['thickness_range'][0] + self.fabrication_parameters['thickness_range'][1] / 2
        avg_thickness = 0.05
        avg_dist = (d_max_start + d_max_end) // 2
        N = int(avg_dist / avg_thickness)
        # N = 100
        d_weight = [[] for _ in range(N)]

        # claculate the weighted distance for each line and each vertex
        for n in range(N):
            n += 1
            t = float(n)/N
            for vertex in self.mesh.vertices():
                dist = t * distances_start[vertex] - (1-t) * distances_end[vertex]
                d_weight[n-1].append(dist)

        # calculate the locations of the zero crossings for each line
        zero_crossings = [[] for _ in range(N)]
        for i, weight in enumerate(d_weight):
            for face in self.mesh.faces():
                vertices = self.mesh.face_vertices(face)
                dists = [weight[v] for v in vertices]
                crossings = self.find_crossings(vertices, dists)
                if crossings != []:
                    zero_crossings[i].extend(crossings)
        
        self.isolines_spherical(self.mesh.vertex_coordinates(start), self.mesh.vertex_coordinates(end), zero_crossings)
        heat_list = [d_weight, distances_start, distances_end]

        return (start, end), heat_list, self.isolines


    def find_center(self, nodes):
        # find the center of the points of the isoline
        x, y, z = 0, 0, 0
        for node in nodes:
            x += node[0]
            y += node[1]
            z += node[2]
        return [x/len(nodes), y/len(nodes), z/len(nodes)]
    
    def barycentric(self, start, triangle, nodes):
        index = 0
        for i, isoline_nodes in enumerate(nodes):
            if len(isoline_nodes) == 0:
                print(index)
                continue
            barycentric_coords = [barycentric_coordinates(start, triangle, node) for node in isoline_nodes]


    def isolines_spherical(self, start, end, nodes):
        index = 0
        for i, isoline_nodes in enumerate(nodes):
            if len(isoline_nodes) == 0:
                print(index)
                continue
            # check which point is closer to the isoline
            center = self.find_center(isoline_nodes)
            if distance(start, center) < distance(end, center):
                origin = start
            else:
                origin = end
            # sort the points on the isoline according to their theta value (spherical coordinate) relative to the new origin
            nodes_factor = [self.spherical_factor(node.x, node.y, node.z, origin) for node in isoline_nodes]
            nodes_sorted = [node for _, node in sorted(zip(nodes_factor, isoline_nodes), key=lambda pair: pair[0])]
            # reverse every second isoline to make sure the lines are connected in the right order
            if i % 2 == 0:
                nodes_sorted.reverse()
            # add the nodes to the network and connect them
            for node in nodes_sorted:
                attr_dict = {
                    'x':node[0], 'y':node[1], 'z':node[2],
                }
                self.isolines.add_node(key=index, attr_dict=attr_dict)
                if index != 0:
                    self.isolines.add_edge(index-1, index)
                index += 1
    

    def spherical_factor(self, x, y, z, origin, result="theta"):
        x, y, z = shift_origin(x, y, z, origin)
        # calculate the spherical coordinate of the point relative to the new origin
        if result == "theta":
            if z > 0:
                result = math.atan2(math.sqrt(x**2 + y**2), z)
            elif z < 0:
                result = math.pi + math.atan2(math.sqrt(x**2 + y**2), z)
            elif z == 0 and x*y != 0:
                result = math.pi / 2
            else:
                result = 0
        elif result == "phi":
            if x > 0:
                result = math.atan2(y, x)
            elif x < 0 and y >= 0:
                result = math.pi + math.atan2(y, x)
            elif x < 0 and y < 0:
                result = -math.pi + math.atan2(y, x)
            elif x == 0 and y > 0:
                result = math.pi / 2
            elif x == 0 and y < 0:
                result = -math.pi / 2
            else:
                result = 0
        elif result == "radius":
            result = magnitude(x, y, z)
        # return the theta value
        return result


    def find_crossings(self, vertices, dists):
        [a, b, c, d] = dists
        [v_a, v_b, v_c, v_d] = vertices
        zero_crossing = []
        # check for crossings and their location in between the circumference of the face
        if math.copysign(1, a) !=  math.copysign(1, b):
            zero_crossing.append(self.calculate_crossings(a, b, v_a, v_b))
        if math.copysign(1, b) !=  math.copysign(1, c):
            zero_crossing.append(self.calculate_crossings(b, c, v_b, v_c))
        if math.copysign(1, c) !=  math.copysign(1, d):
            zero_crossing.append(self.calculate_crossings(c, d, v_c, v_d))
        if math.copysign(1, d) !=  math.copysign(1, a):
            zero_crossing.append(self.calculate_crossings(d, a, v_d, v_a))
        return zero_crossing

    
    def calculate_crossings(self, start, end, start_vertice, end_vertice):
        # calculate the factor of the distance between the two vertices (0-1)
        factor = -start / ((end-start))
        vector = Vector.from_start_end(self.mesh.vertex_coordinates(start_vertice), self.mesh.vertex_coordinates(end_vertice))
        [x, y, z] = self.mesh.vertex_coordinates(start_vertice)
        return Point(x, y, z) + vector.scaled(factor)


    def connect_isolines(self, start, nodes):
        index = 0
        for i, isoline_nodes in enumerate(nodes):
            if len(isoline_nodes) == 0:
                print(index)
                continue
            # calculating the center of the points of the isoline
            center = self.find_center(isoline_nodes)
            # calculate the two endpoints by finding the farthest points from the center 
            # and choosing the one closer to the start point (the end point of the previous isoline)
            kd = KDTree(isoline_nodes)
            nodes_from_center = kd.nearest_neighbors(center, len(isoline_nodes), True)
            not_end_nodes = [n[1] for n in nodes_from_center[:-2]]
            coord_prev_node, prev_node, d = kd.nearest_neighbor(start, exclude=not_end_nodes)
            attr_dict = {
                'x':coord_prev_node[0], 'y':coord_prev_node[1], 'z':coord_prev_node[2],
            }
            self.isolines.add_node(key=index, attr_dict=attr_dict)
            # do not add a line from the start point to the first isoline point
            if index != 0:
                self.isolines.add_edge(index-1, index)
            index += 1
            # while there are still nodes in the isoline, find the nearest node to the previous node
            # and remove the previous node from the search
            exclude_nodes = [prev_node]
            while len(exclude_nodes) < len(isoline_nodes):
                coord_node, node, d = kd.nearest_neighbor(coord_prev_node, exclude=exclude_nodes)
                attr_dict = {
                    'x':coord_node[0], 'y':coord_node[1], 'z':coord_node[2],
                }
                self.isolines.add_node(key=index, attr_dict=attr_dict)
                self.isolines.add_edge(index-1, index)
                index += 1
                # set the current node as the previous node for the next iteration
                exclude_nodes.append(node)
                coord_prev_node = coord_node
                prev_node = node
            # set the start point of the next isoline as the previous node
            start = coord_prev_node