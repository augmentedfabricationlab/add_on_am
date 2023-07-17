import math


from compas.utilities import linspace
from compas.datastructures import Network, network_polylines
from compas.geometry import Frame, Vector, Point, Translation, KDTree, angle_vectors, cross_vectors, distance_point_point, dot_vectors
from compas.colors import Color, ColorMap

class SegmentPathPlanner():
    def __init__(self):
        self.mesh = None
        self.network = Network(name="network")
        self.network.path = []
        self.network.default_node_attributes = {
            'x':0, 'y':0, 'z':0,
            'vx':0, 'vy':0, 'vz':0,
            'r':0, 'g':0, 'b':0,
            'color': None,
            'force': 0,
            'skip' : False,
            'neighbors':0,
            'number_of_neighbors':0,
            'sphere':None,
            'frame':None,
            'thickness':0.0,
            'radius':0.0,
            'area':0.0,
            'velocity':0.0,
            'nozzle_distance':0.0,
            'tool_frame':None
            }
        self.fabrication_parameters = {
            'material_flowrate':500.0,          # l/hr
            'thickness_range':[0.008, 0.022],     # m
            'radius_range':[0.040, 0.075],        # m
            'distance_range':[0.100, 0.300],      # m
            'measured_radii':[0.0625, 0.075],        # [m]
            'measured_distances':[0.150, 0.300],  # [m]
            'measured_thicknesses':[0.012, 0.009], # [m]
        }
        self.color_map=None
        self.thickness_map=None

    def set_quad_mesh(self, mesh):
        self.mesh = mesh
        return self.mesh

    def set_quad_mesh_from_rhinomesh(self, rhinomesh):
        self.mesh = rhinomesh.to_compas()
        self.mesh.update_default_vertex_attributes(attr_dict={'c':Color(1,1,1)})
        vertexcolors = rhinomesh.geometry.VertexColors
        if len(vertexcolors) != 0:
            for i, _ in enumerate(rhinomesh.geometry.Vertices):
                self.mesh.vertex_attributes(i, ["c"], [Color.from_rgb255(vertexcolors[i].R, vertexcolors[i].G, vertexcolors[i].B)])
        return self.mesh
    
    def face_color(self, key):
        vertices = self.mesh.face_vertices(key)
        # center = self.face_center(key)
        vtx_colors = [self.mesh.vertex_attributes(vertex, ["c"])[0] for vertex in vertices]
        r = (vtx_colors[0].r + vtx_colors[1].r + vtx_colors[2].r + vtx_colors[3].r)/4
        g = (vtx_colors[0].g + vtx_colors[1].g + vtx_colors[2].g + vtx_colors[3].g)/4
        b = (vtx_colors[0].b + vtx_colors[1].b + vtx_colors[2].b + vtx_colors[3].b)/4
        face_color = Color(r,g,b)
        return face_color

    def create_quad_mesh_from_surface(self, surface, nu, nv):
        self.mesh = surface.to_compas_mesh()
        return self.mesh

    def set_network_nodes(self, total_network, nodes):
        # get coordinates, normal, neighbors and color from total_network and pass them to self.network for each node in nodes
        # for node in nodes:
        #     point = total_network.node_coordinates(key=node)
        #     normal = total_network.node_attributes(key=node, name=["vx","vy","vz"])
        #     neighbors = total_network.node_attribute(key=node, name='neighbors')
        #     color = total_network.node_attribute(key=node, name='color')
        #     attr_dict = {
        #         'x':point[0], 'y':point[1], 'z':point[2],
        #         'vx':normal[0], 'vy':normal[1], 'vz':normal[2],
        #         'r':color.rgb255[0], 'g':color.rgb255[1], 'b':color.rgb255[2],
        #         'color': color,
        #         'force': 0,
        #         'skip' : False,
        #         'neighbors':neighbors,
        #         'number_of_neighbors':len(neighbors),
        #         'sphere':None,
        #         'frame':None,
        #         'thickness':0.0,
        #         'radius':0.0,
        #         'area':0.0,
        #         'velocity':0.0,
        #         'nozzle_distance':0.0,
        #         'tool_frame':None
        #     }
        #     self.network.add_node(key=node, attr_dict=attr_dict)
        
        for index in nodes:
            self.add_node(index, nodes=nodes)
        
        


    def add_node(self, index, attr_dict={}, nodes=[], **kwattr):
        point = self.mesh.face_center(index)
        normal = self.mesh.face_normal(index)
        neighbors = []
        for neighbor in self.mesh.face_neighbors(index):
            if neighbor in nodes:
                neighbors.append(neighbor)
        color = self.face_color(index)
        attr_dict.update({
            'x':point[0], 'y':point[1], 'z':point[2],
            'vx':normal[0], 'vy':normal[1], 'vz':normal[2],
            'r':color.rgb255[0], 'g':color.rgb255[1], 'b':color.rgb255[2],
            'color': color,
            'force': 0,
            'skip' : False,
            'neighbors':neighbors,
            'number_of_neighbors':len(neighbors),
            'sphere':None,
            'frame':None,
            'thickness':0.0,
            'radius':0.0,
            'area':0.0,
            'velocity':0.0,
            'nozzle_distance':0.0,
            'tool_frame':None
        })
        attr_dict.update(**kwattr)
        self.network.add_node(key=index, attr_dict=attr_dict)
    
    def add_edge(self, start, end):
        new_edge = self.network.add_edge(start, end)
        # if self.network.node_attribute(key=start, name='frame') is None:
        #     self.set_node_frame_from_edge(start, new_edge)
        # self.set_node_frame_from_edge(end, new_edge)
        # if self.network.node_attribute(key=start, name='frame') is None:
        #     self.select_node_frame(start)
        # self.select_node_frame(end)
    
    def set_node_frame(self, node, flip):
        norm = Vector.from_data(self.network.node_attributes(key=node, names=['vx','vy','vz']))
        if flip:
            norm = norm.inverted()
        v1 = cross_vectors(norm, Vector.Zaxis()) # Zaxis
        v2 = cross_vectors(norm,v1)
        frame = Frame(Point.from_data(self.network.node_coordinates(node)), v1, v2)
        self.network.node_attribute(key=node, name='frame', value=frame)
        return frame
    
    def map_reachability(self, map3d, center):
        trans = Translation.from_vector(center)
        cloud = Point.transformed_collection(map3d.points, trans)
        kd = KDTree(cloud)
        for node, point in enumerate(self.network.to_points()):
            p, k, dist = kd.nearest_neighbor(point)
            sp = map3d.spheres[k].copy()
            sp.point = point
            self.network.node_attribute(key=node, name='sphere', value=sp)
    
    def select_node_frame(self, node):
        normal = Vector.from_data(self.network.node_attributes(key=node, names=['vx','vy','vz']))
        # if dot_vectors(normal, -Vector.Zaxis())<0:
        #     norm = normal.inverted()
        angles = []
        for pose in self.network.node_attribute(key=node, name='sphere').poses:
            angles.append(angle_vectors(normal, pose.normal))
        pose = self.network.node_attribute(key=node, name='sphere').poses[angles.index(min(angles))]
        pose.point = Point.from_data(self.network.node_coordinates(node))
        self.network.node_attribute(key=node, name='frame', value=pose)

    def add_force(self, force):
        for i in self.mesh.faces():
            n = (force[0][i] + force[1][i]) /2
            self.network.node_attribute(key=i, name='force', value=n)

    def get_node(self, **kwargs):
        """
        **kwargs can include any of the following:
        - any condition to find nodes based on network.default_node_attributes
        - 'orientation' : either 'x', 'y', or 'z'
        - 'index' : 0 or 1 for minimum or maximum value
        """
        attr_dict = {
            'func' : 0,
            'orientation' : 'x',
            'idx' : 0
        }
        attr_dict.update(kwargs)
        # print(attr_dict)
        conditions = {}
        func_dict = {0:min,1:max,2:sorted}
        func = func_dict[attr_dict['func']]
        for key, value in kwargs.items():
            if key in self.network.default_node_attributes.keys():
                conditions.update({key:value})
       #  print(conditions)
        # if 'orientation' not in locals():
        #     orientation = None
        # if 'index' not in locals():
        #     func = None

        nodes = list(self.network.nodes_where(conditions))
        if nodes != []:
            vals = [self.network.node_attribute(key=k, name=attr_dict['orientation']) for k in nodes]
            # print(vals)
            fval = func(vals)
            if isinstance(fval, list):
                fval = fval[attr_dict['idx']]
            return nodes[vals.index(fval)]
        else:
            # In case there are valid nodes, returns Nonetype
            return None

    def lowest_axis_path(self, orientation="z"):
        """Creates a tool-path based on the given mesh topology.

        Args:
            mesh (compas mesh): Description of `mesh`
            orientation (int): X-axis = 1, Y-axis = 2, Z-axis =3
        """
        if self.mesh == None:
            raise ValueError
        if self.network == None:
            self.set_network_nodes()

        n = 0 # Number of interruptions        
        current = self.get_node(number_of_neighbors=2, orientation=orientation, func=2, idx=0)
                    
        # Getting the starting point
        print("start: " + str(current))
        # Path finding process
        for index in self.mesh.faces():
            # Look for the neighbor with the lowest x/y/z
            self.network.path.append(current)
            neighbornodes = {}
            # print(self.network.node_attribute(key=current, name='neighbors'))
            for i in self.network.node_attribute(key=current, name='neighbors'):
                if len(self.network.connected_edges(key=i))==0:
                    neighbornodes.update({i:self.network.node_attribute(key=i, name=orientation)})
            # If neighbors found then find the closest one based on orientation
            if neighbornodes != {}:
                if len(list(neighbornodes.keys()))>2:
                    following = list(neighbornodes.keys())[list(neighbornodes.values()).index(sorted(list(neighbornodes.values()))[1])]
                else:
                    following = list(neighbornodes.keys())[list(neighbornodes.values()).index(min(list(neighbornodes.values())))]
                # Draw a line between the current and following face centerpoints
                self.add_edge(current, following)
            # If the face doesn't have free neighbors
            else:
                # But has remaining unconnected nodes
                if self.network.number_of_nodes()-1 != self.network.number_of_edges():
                    # Move to the closest available face centerpoint
                    following = self.move_to_closest(current)
                    if following == current:
                        return self.network, n
                    # Draw a line between the current and the following face
                    self.add_edge(current, following)
                    n += 1
            current = following
            print(current)
        return self.network, n
    

    def from_heat_method(self, points):
        if self.mesh == None:
            raise ValueError
        mesh_face_centers = [self.mesh.face_center(index) for index in self.mesh.faces()]
        kd = KDTree(mesh_face_centers)
        for index, point in enumerate(points):
            p, node_index, dist = kd.nearest_neighbor(point)
            normal = self.mesh.face_normal(node_index)
            attr_dict = {
                'x':point[0], 'y':point[1], 'z':point[2],
                'vx':normal[0], 'vy':normal[1], 'vz':normal[2],
                'force': 0,
                'sphere':None,
                'frame':None,
                'thickness':0.0,
                'radius':0.0,
                'area':0.0,
                'velocity':0.0,
                'nozzle_distance':0.0,
                'tool_frame':None
            }
            self.network.add_node(key=index, attr_dict=attr_dict)
            self.network.path.append(index)
            if index != 0:
                self.network.add_edge(index-1, index)
            print(index)
        return self.network



    def move_to_closest(self, current):
        current_point = Point.from_data(self.network.node_coordinates(key=current))
        distances = {}
        nodes = [key for key in self.network.nodes() if len(self.network.connected_edges(key))==0]
        # print(nodes)
        for j in nodes:
            if self.network.node_attribute(key=j, name='skip') == False:
                d = distance_point_point(current_point, Point.from_data(self.network.node_coordinates(key=j)))
                distances.update({j:d})
        if len(list(distances.keys())) == 0:
            return current
        min_d = min(distances.values())
        following = distances.keys()[distances.values().index(min_d)]
        return following

    def set_fabrication_parameters(self, *args, **kwargs):
        '''
        *args : dict {}
        **kwargs: keyword arguments and values
        '''
        self.fabrication_parameters.update(args)
        for key, value in kwargs.items():
            self.fabrication_parameters[key] = value

    def set_color_map(self, colors=None, color_map=None, rangetype="full"):
        if colors is not None:
            if len(colors)==1:
                color_map = ColorMap.from_color(colors[0], rangetype)
            elif len(colors)==2:
                color_map = ColorMap.from_two_colors(*colors)
            elif len(colors)==3:
                color_map = ColorMap.from_three_colors(*colors)
            elif len(colors)>3:
                raw_colors = []
                for color in colors:
                    raw_colors.append(color.rgb())
                color_map = ColorMap(raw_colors)
        self.color_map = color_map

    def set_thickness_map(self, thicknesses):
        thickness_map = []
        if len(thicknesses)==2:
            for i in linspace(0,1.0,256):
                thickness_map.append(thicknesses[0]*(1-i)+thicknesses[1]*i)
        elif len(thicknesses)==3:
            for i in linspace(0,1.0,128):
                thickness_map.append(thicknesses[0]*(1-i)+thicknesses[1]*i)
            for i in linspace(0,1.0,128):
                thickness_map.append(thicknesses[1]*(1-i)+thicknesses[2]*i)
        elif len(thicknesses)>3:
            thickness_map.extend(thicknesses)
        self.thickness_map = thickness_map

    def calculate_fabrication_parameters(self, num_layers, n_layer, scale, measured=True):
        nodes = [key for key in self.network.nodes() if len(self.network.connected_edges(key))!=0]
        for node in nodes:
            self.set_node_area_radius(node, scale)
            self.set_node_thickness(node, num_layers)
            self.set_node_distance(node, num_layers, n_layer, measured)
            self.set_node_velocity(node)

    def set_node_area_radius(self, node, scale):
        area = self.mesh.face_area(node)
        self.network.node_attribute(key=node, name='area', value=(scale**2)*area)
        self.network.node_attribute(key=node, name='radius', value=scale*math.sqrt(area/math.pi))

    def set_node_distance(self, node, num_layers, n_layer, measured=True):
        radius = self.network.node_attribute(key=node, name='radius')
        if measured:
            drange = self.fabrication_parameters['measured_distances']
            rrange = self.fabrication_parameters['measured_radii']
        else:
            drange = self.fabrication_parameters['distance_range']
            rrange = self.fabrication_parameters['radius_range']

        distance = ((drange[1]-drange[0])/(rrange[1]-rrange[0]))*radius
        distance_thickness = self.network.node_attribute(key=node, name='thickness')*(n_layer/num_layers)
        self.network.node_attribute(key=node, name='distance', value=distance)
        frame = self.network.node_attribute(node, 'frame')
        T = Translation.from_vector(frame.zaxis*-(distance+distance_thickness))
        tool_frame = frame.transformed(T)
        self.network.node_attribute(key=node, name='tool_frame', value=tool_frame)

    def set_node_thickness(self, node, num_layers):
        node_color = self.network.node_attribute(node, 'color').rgb255
        n=0
        while n < 255:
            ncol = self.network.node_attribute(n, 'color').rgb255
            if ncol == node_color:
                break
            n+=1
        t = self.thickness_map[n]
        self.network.node_attribute(key=node, name='thickness', value=t/num_layers)

    def set_node_velocity(self, node):
        volume = self.network.node_attribute(key=node, name='area')*self.network.node_attribute(key=node, name='thickness')
        velocity = self.network.node_attribute(key=node, name='radius')/(volume/(self.fabrication_parameters['material_flowrate']/60))
        self.network.node_attribute(key=node, name='velocity', value=velocity)
    
    def node_color(self, node, color_type='rgb'):
        return self.network.node_attribute(node, 'color').rgb255
