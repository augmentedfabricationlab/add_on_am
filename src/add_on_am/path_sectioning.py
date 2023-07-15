import math


from compas.utilities import linspace
from compas.datastructures import Network
from compas.geometry import Frame, Vector, Point, Translation, cross_vectors, distance_point_point
from compas.colors import Color, ColorMap


class SurfacePathPlanner_two():
    def __init__(self, wall, resolution, envelope):
        self.mesh = None
        self.envelope = envelope
        self.path_network = Network(name="path_network")
        self.path_network.path = []
        self.path_network.default_node_attributes = {
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

        self.pos_network = Network(name="pos_network")
        self.pos_network.path = []
        self.pos_network.default_node_attributes = {
            'x':0, 'y':0, 'z':0,
            "ri": 0,
            "vertices": [],
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
        
        self.resolution = resolution
        self.x_size = int(wall.width / resolution) + 1
        self.wall = wall
        self.set_quad_mesh(wall.mesh)

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

    def create_quad_mesh_from_surface(self, surface, nu, nv):
        self.mesh = surface.to_compas_mesh()
        return self.mesh

    def create_pos_network(self):
        index = 0
        for i in range(self.x_size):
            x = i*self.resolution
            y_down = -self.wall.sin_wave(self.wall.down_amp, self.wall.down_freq, self.wall.down_phase, x)
            y_up = -self.wall.sin_wave(self.wall.up_amp, self.wall.up_freq, self.wall.up_phase, x)
            dynamic_reach = int((self.envelope.r_out - self.envelope.r_in + abs(y_down-y_up)) / self.resolution) + 1
            for j in range(dynamic_reach):
                y_l = y_down + self.envelope.r_in + j*self.resolution
                y_r = y_down - self.envelope.r_in - j*self.resolution
                attr_dict = {
                    'x':x, 'y':y_l, 'z':0,
                    "ri": 0,
                    "vertices": [],
                    }
                self.pos_network.add_node(key=index, attr_dict=attr_dict)
                index += 1
                attr_dict = {
                    'x':x, 'y':y_r, 'z':0,
                    "ri": 0,
                    "vertices": [],
                    }
                self.pos_network.add_node(key=index, attr_dict=attr_dict)
                index += 1
        return self.pos_network

    def set_network_nodes(self):
        for index in self.mesh.faces():
            self.add_node(index)


    def add_node(self, index, attr_dict={}, **kwattr):
        point = self.mesh.face_center(index)
        normal = self.mesh.face_normal(index)
        neighbors = self.mesh.face_neighbors(index)
        attr_dict.update({
            'x':point[0], 'y':point[1], 'z':point[2],
            'vx':normal[0], 'vy':normal[1], 'vz':normal[2],
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
        self.path_network.add_node(key=index, attr_dict=attr_dict)
    
    def add_edge(self, start, end):
        new_edge = self.path_network.add_edge(start, end)
        # if self.network.node_attribute(key=start, name='frame') is None:
        #     self.set_node_frame_from_edge(start, new_edge)
        # self.set_node_frame_from_edge(end, new_edge)
        # if self.network.node_attribute(key=start, name='frame') is None:
        #     self.select_node_frame(start)
        # self.select_node_frame(end)
    
    def set_node_frame(self, node, flip):
        norm = Vector.from_data(self.path_network.node_attributes(key=node, names=['vx','vy','vz']))
        if flip:
            norm = norm.inverted()
        v1 = cross_vectors(norm, Vector.Zaxis()) # Zaxis
        v2 = cross_vectors(norm,v1)
        frame = Frame(Point.from_data(self.path_network.node_coordinates(node)), v1, v2)
        self.path_network.node_attribute(key=node, name='frame', value=frame)
        return frame

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
            if key in self.path_network.default_node_attributes.keys():
                conditions.update({key:value})
       #  print(conditions)
        # if 'orientation' not in locals():
        #     orientation = None
        # if 'index' not in locals():
        #     func = None

        nodes = list(self.path_network.nodes_where(conditions))
        if nodes != []:
            vals = [self.path_network.node_attribute(key=k, name=attr_dict['orientation']) for k in nodes]
            fval = func(vals)
            if isinstance(fval, list):
                fval = fval[attr_dict['idx']]
            return nodes[vals.index(fval)]
        else:
            # In case there are valid nodes, returns Nonetype
            return None
    
    def reachability(self, reachability_map, kd, pos_x, pos_y, node_x, node_y, node_z):
        # trans = Translation.from_vector(Vector(pos_x, pos_y, 0))
        # cloud = Point.transformed_collection(reachability_map.points, trans)
        # kd = KDTree(reachability_map.points)
        if reachability_map:
            p, k, dist = kd.nearest_neighbor(Point(node_x-pos_x, node_y-pos_y, node_z))
            return reachability_map.spheres[k].ri
        else:
            return 100
        

    def linear_reach(self, node_x, node_y, node_z):
        if node_z > self.envelope.h_out+self.envelope.z:
            d = math.sqrt((node_x-self.envelope.x)**2 + (node_y-self.envelope.y)**2 + (node_z-self.envelope.h_out-self.envelope.z)**2)
        elif node_z < self.envelope.z:
            d = math.sqrt((node_x-self.envelope.x)**2 + (node_y-self.envelope.y)**2 + (node_z-self.envelope.z)**2)
        else:
            d = math.sqrt((node_x-self.envelope.x)**2 + (node_y-self.envelope.y)**2)

        t = 0
        if self.envelope.r_in < d < self.envelope.r_out:
            s_max = self.envelope.r_out - self.envelope.r_in
            s = 2*abs(d - self.envelope.r_in - s_max/2)/s_max
            t = 1 - s
            
        return 100*t
    

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
    
        # returns poses of all nodes that are poses ordered along the path
    def poses(self, nodes, pos):
        # if pos is on other side flip normals
        flip = False
        if pos.Y < self.path_network.node_attribute(nodes[0], "y"):
            flip = True

        frames = []
        for node in nodes:
            frames.append(self.set_node_frame(node, flip))        
        return frames

    def positions_in_reach(self, node, previous_pos):
            reachable_pos = []
            # looping over the positions from which the previous nodes were reachable and appending them to the next iteration if current node is also reachable
            for pos in previous_pos:
                [self.envelope.x, self.envelope.y] = self.pos_network.node_attributes(pos, ["x", "y"])
                [x, y, z] = self.path_network.node_attributes(node, ["x", "y", "z"])
                
                inside = self.envelope.point_inside(x, y, z)
                if inside:
                    reachable_pos.append(pos)


            if len(reachable_pos) == 1:
                # if there is no more position from which the current node can be reached, add the previous position to the dict and start new with current node
                
                # check if final node is reached
                if node == list(self.mesh.faces())[-1]:
                    print("last node: ", node, previous_pos)
                    return previous_pos, True
                print(previous_pos)
                return reachable_pos[0], True
            else:
                return reachable_pos, False

    def section_path(self, pos=None, min_ri=0):
        """Creates a tool-path based on the given mesh topology.

        Args:
            mesh (compas mesh): Description of `mesh`
            orientation (int): X-axis = 1, Y-axis = 2, Z-axis =3
        """
        if self.mesh == None:
            raise ValueError
        if self.path_network == None:
            self.set_network_nodes()
        if self.pos_network == None:
            raise ValueError

        n = 0 # Number of interruptions        
        current = self.get_node(number_of_neighbors=2, orientation="z", func=2, idx=1)
        if pos == None:
            pos_set = False
            positions, segment = self.positions_in_reach(current, list(self.pos_network.nodes()))
        else:
            self.envelope.x = pos.X
            self.envelope.y = pos.Y
            pos_set = True
            segment = False
        direction = "x"
        reverse = False
                    
        # Getting the starting point
        print(current)
        # Path finding process
        for face in self.mesh.faces():
            # Look for the neighbor with the lowest x/y/z
            self.path_network.path.append(current)
            neighbornodes = {}
            neighbornodes_z = {}
            for i in self.path_network.node_attribute(key=current, name='neighbors'):
                if len(self.path_network.connected_edges(key=i))!=0:
                    continue
                [x, y, z] = self.path_network.node_attributes(i, ["x", "y", "z"])
                if pos_set:
                    inside = self.envelope.point_inside(x, y, z)
                else:
                    inside = True
                

                if inside:
                    neighbornodes[i] = self.path_network.node_attribute(key=i, name=direction) - self.path_network.node_attribute(key=current, name=direction)
                    neighbornodes_z[i] = self.path_network.node_attribute(key=i, name="z")
                else:
                    print("not inside envelope: " + str(i))
                
            # If neighbors found then find the closest one based on orientation
            if neighbornodes != {}:
                while len(neighbornodes.keys())>2:
                    del neighbornodes[max(neighbornodes_z, key=neighbornodes_z.get)]
                
                # if only neighbor above and below are reachable, delete the one below, so it is clear that the path is going up
                if len(neighbornodes.keys()) == 2:
                    if abs(neighbornodes.values()[0] - neighbornodes.values()[1]) < 0.01:
                        del neighbornodes[min(neighbornodes_z, key=neighbornodes_z.get)]

                if reverse:
                    following = min(neighbornodes, key=neighbornodes.get)
                elif not reverse:
                    following = max(neighbornodes, key=neighbornodes.get)
                # if position is not yet final use threshold to move upwards if to low
                if not pos_set and len(positions) < len(list(self.pos_network.nodes())):
                    ri_temp = []
                    for position in positions:
                        [self.envelope.x, self.envelope.y] = self.pos_network.node_attributes(position, ["x", "y"])
                        ri_temp.append(self.linear_reach(x, y, z))
                    print("average_ri: " + str(sum(ri_temp)/len(ri_temp)), "min_ri: " + str(min(ri_temp)), "max_ri: " + str(max(ri_temp)))
                    if min(ri_temp) < min_ri:
                        following = max(neighbornodes_z, key=neighbornodes_z.get)
                    

                # ({1205: 5.2692638519147295e-07, 1203: -5.1641029763516144e-07}, {1205: 0.27494289168354835, 1203: 0.17494203073187164})
                # this is not reliable and has to be changed
                # if neighbornodes_z[following] - self.path_network.node_attribute(key=current, name="z") < -0.02:
                #     del neighbornodes[following]
                #     following = neighbornodes.keys()[0]
                
                # if path is going up reverse direction
                x_cond = abs(self.path_network.node_attribute(key=current, name="x")-self.path_network.node_attribute(key=following, name="x")) < 0.01
                z_cond = self.path_network.node_attribute(key=following, name="z") - self.path_network.node_attribute(key=current, name="z") > 0.03
                if x_cond and z_cond:
                    reverse = not reverse
                    print("reverse: " + str(reverse) + "  " + str(current))
                
                if not pos_set:
                    positions, segment = self.positions_in_reach(following, positions)

                # Draw a line between the current and following face centerpoints
                self.add_edge(current, following)

                if segment:
                    print("position: " + str(positions))
                    print(neighbornodes)
                    pos_set = True
                    [self.envelope.x, self.envelope.y] = self.pos_network.node_attributes(positions, ["x", "y"])
                    segment = False

                    # positions, segment = self.positions_in_reach(following, list(self.pos_network.nodes()))

            
            # If the face doesn't have free neighbors
            else:
                # start new and find another position
                print("no neighbors")
                return self.path_network, positions
                # But has remaining unconnected nodes
                if self.path_network.number_of_nodes()-1 != self.path_network.number_of_edges():
                    # Move to the closest available face centerpoint
                    following = self.move_to_closest(current)
                    if following == current:
                        return self.path_network, n
                    # Draw a line between the current and the following face
                    self.add_edge(current, following)
                    n += 1
            current = following
            #print(current)
        return self.path_network, positions

    def move_to_closest(self, current):
        current_point = Point.from_data(self.path_network.node_coordinates(key=current))
        distances = {}
        nodes = [key for key in self.path_network.nodes() if len(self.path_network.connected_edges(key))==0]
        # print(nodes)
        for j in nodes:
            if self.path_network.node_attribute(key=j, name='skip') == False:
                d = distance_point_point(current_point, Point.from_data(self.path_network.node_coordinates(key=j)))
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
        nodes = [key for key in self.path_network.nodes() if len(self.path_network.connected_edges(key))!=0]
        for node in nodes:
            self.set_node_area_radius(node, scale)
            self.set_node_thickness(node, num_layers)
            self.set_node_distance(node, num_layers, n_layer, measured)
            self.set_node_velocity(node)

    def set_node_area_radius(self, node, scale):
        area = self.mesh.face_area(node)
        self.path_network.node_attribute(key=node, name='area', value=(scale**2)*area)
        self.path_network.node_attribute(key=node, name='radius', value=scale*math.sqrt(area/math.pi))

    def set_node_distance(self, node, num_layers, n_layer, measured=True):
        radius = self.path_network.node_attribute(key=node, name='radius')
        if measured:
            drange = self.fabrication_parameters['measured_distances']
            rrange = self.fabrication_parameters['measured_radii']
        else:
            drange = self.fabrication_parameters['distance_range']
            rrange = self.fabrication_parameters['radius_range']

        distance = ((drange[1]-drange[0])/(rrange[1]-rrange[0]))*radius
        distance_thickness = self.path_network.node_attribute(key=node, name='thickness')*(n_layer/num_layers)
        self.path_network.node_attribute(key=node, name='distance', value=distance)
        frame = self.path_network.node_attribute(node, 'frame')
        T = Translation.from_vector(frame.zaxis*-(distance+distance_thickness))
        tool_frame = frame.transformed(T)
        self.path_network.node_attribute(key=node, name='tool_frame', value=tool_frame)

    def set_node_thickness(self, node, num_layers):
        node_color = self.path_network.node_attribute(node, 'color').rgb255
        n=0
        while n < 255:
            ncol = self.path_network.node_attribute(n, 'color').rgb255
            if ncol == node_color:
                break
            n+=1
        t = self.thickness_map[n]
        self.path_network.node_attribute(key=node, name='thickness', value=t/num_layers)

    def set_node_velocity(self, node):
        volume = self.path_network.node_attribute(key=node, name='area')*self.path_network.node_attribute(key=node, name='thickness')
        velocity = self.path_network.node_attribute(key=node, name='radius')/(volume/(self.fabrication_parameters['material_flowrate']/60))
        self.path_network.node_attribute(key=node, name='velocity', value=velocity)
    
    def node_color(self, node, color_type='rgb'):
        return self.path_network.node_attribute(node, 'color').rgb255
