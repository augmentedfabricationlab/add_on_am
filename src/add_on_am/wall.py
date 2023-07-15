import math

from compas.datastructures import Mesh
from compas_rhino.geometry import RhinoMesh
import Rhino.Geometry as rg


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

    def remesh(self):
        mesher = Remesher(self.mesh, resolution=0.05)
        mesher.add_nodes(self.z_size, self.x_size)
        mesher.shift_nodes()
        distances = mesher.distances()
        return mesher.mesh

class NewWall(Wall):
    def __init__(self, width=2.0, height=2.0, elsize=0.025, rotation=0.0):
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
    
    def undulate(self, up=None, down=None):
        self.up_amp = up.amp
        self.up_freq = up.freq
        self.up_phase = up.ro
        self.down_amp = down.amp
        self.down_freq = down.freq
        self.down_phase = down.ro
        

    def from_brep(self, inputBrep, max_edge_length=0.05, min_edge_length=0.05):
        # Create a compas mesh from an input Brep surface
        par = rg.MeshingParameters()
        #par.GridMinCount = density*10
        par.MinimumEdgeLength = max_edge_length
        par.MaximumEdgeLength = min_edge_length
        meshes = rg.Mesh.CreateFromBrep(inputBrep, par)
        brepMesh = rg.Mesh()
        for mesh in meshes:
            brepMesh.Append(mesh)
        meshsrf = RhinoMesh.from_geometry(brepMesh)
        mesh = meshsrf.to_compas()

        # In case the mesh isn't a quadmesh, apply a remesh method
        if mesh.is_quadmesh() == False:
            quad_p = rg.QuadRemeshParameters()
            quad_p.AdaptiveSize = 0
            quad_p.TargetEdgeLength = self.elsize
            brepMesh = rg.Mesh.QuadRemesh(brepMesh, quad_p)

        # Turn into a final quad compas mesh
        meshsrf = RhinoMesh.from_geometry(brepMesh)
        mesh = meshsrf.to_compas()

        return mesh
    
class NewMesher:
    def __init__(self, mesh, resolution=0.05):
        self.distances = {}
        self.mesh = mesh
        self.resolution = resolution
        self.mesh_list = [self.mesh.vertex_coordinates(v) for v in self.mesh.vertices()]
    
    def shift_nodes(self):
        for vertex in self.mesh.vertices():
            # finding x-neighbor
            neighbors = self.mesh.vertex_neighbors(vertex)
            x, y, z = self.mesh.vertex_coordinates(vertex)
            x_n = [self.mesh.vertex_coordinates(n)[0] for n in neighbors]
            # check if there is a next node in x direction
            if len(x_n) < 4:
                x_dir = False
                for neighbors_x in x_n:
                    if neighbors_x - x > 0.02:
                        x_dir = True
                if not x_dir:
                    # print(vertex, x_n)
                    continue
            next_n = neighbors[x_n.index(max(x_n))]
            # calculating distance between nodes
            x_n, y_n, z_n = self.mesh.vertex_coordinates(next_n)
            distance = math.sqrt((x-x_n)**2 + (y-y_n)**2 + (z-z_n)**2)
            scale = self.resolution/distance
            delta = [x_n - x, y_n - y, z_n - z]
            if not (self.resolution-0.01 < distance < self.resolution+0.01) and distance < 2*self.resolution:
                for i, d in enumerate(delta):
                    delta[i] = d * (scale - 1)
                # shifting node
                #self.mesh.vertex_attributes(next_n, ["x", "y", "z"], values=[self.mesh.vertex_attribute(next_n, "x") + delta[0], self.mesh.vertex_attribute(next_n, "y") + delta[1], self.mesh.vertex_attribute(next_n, "z") + delta[2]])
                self.mesh_list[next_n] = [self.mesh_list[next_n][0] + delta[0], self.mesh_list[next_n][1] + delta[1], self.mesh_list[next_n][2] + delta[2]]
            elif distance > 2*self.resolution:
                for i, d in enumerate(delta):
                    delta[i] = d * scale
                self.mesh_list[next_n] = [self.mesh_list[vertex][0] + delta[0]*2, self.mesh_list[vertex][1] + delta[1]*2, self.mesh_list[vertex][2] + delta[2]*2]
                #self.mesh_list.insert(next_n, [self.mesh_list[vertex][0] + delta[0], self.mesh_list[vertex][1] + delta[1], self.mesh_list[vertex][2] + delta[2]])
                

    def regen(self):
        self.mesh_list.sort(key=lambda z: z[2])
        ordered_list = [[self.mesh_list[0]]]
        k = 0
        for i, vertex in enumerate(self.mesh_list[1:]):
            if vertex[2] - self.mesh_list[i][2] > 0.03:
                k += 1
                ordered_list.append([])
            ordered_list[k].append(vertex)
        print(len(ordered_list))
        for row in ordered_list:
            print(len(row))
        for i, row in enumerate(ordered_list):
            ordered_list[i].sort(key=lambda x: x[0])
        new_mesh = Mesh()
        for j, row in enumerate(ordered_list):
            for i, vertex in enumerate(row):
                new_mesh.add_vertex(x=vertex[0], y=vertex[1], z=vertex[2], i=i, j=j, glob_id=i*len(row)+j, x_disp=0, z_disp=0)
        for row in range(len(ordered_list) - 1):
            for col in range(len(ordered_list[row]) - 1):
                previous_rows = sum([len(ordered_list[i]) for i in range(row)])
                a = previous_rows + col
                b = a + len(ordered_list[row])
                c = b + 1
                d = a + 1
                print(a, b, c, d)
                new_mesh.add_face([a, b, c, d])
        return new_mesh
            
            


class Remesher:
    def __init__(self, mesh, resolution=0.05):
        self.distances = {}
        self.mesh = mesh
        self.resolution = resolution
        
    def add_nodes(self, z_limit, x_limit):
        # adding additional nodes to the mesh where the distance between two nodes is bigger than 2 x resolution
        additional_nodes = {}
        for vertex in self.mesh.vertices():
            # finding x-neighbor
            neighbors = self.mesh.vertex_neighbors(vertex)
            x, y, z = self.mesh.vertex_coordinates(vertex)
            x_n = [self.mesh.vertex_coordinates(n)[0] for n in neighbors]
            # check if there is a next node in x direction
            if len(x_n) < 4:
                x_dir = False
                for neighbors_x in x_n:
                    if neighbors_x - x > 0.02:
                        x_dir = True
                if not x_dir:
                    print(vertex, x_n)
                    continue
            next_n = neighbors[x_n.index(max(x_n))]
            # calculating distance between nodes
            x_n, y_n, z_n = self.mesh.vertex_coordinates(next_n)
            distance = math.sqrt((x-x_n)**2 + (y-y_n)**2 + (z-z_n)**2)
            scale = self.resolution/distance
            delta = [x_n - x, y_n - y, z_n - z]
            # if distance is bigger than 2 x resolution, add a node in between
            if distance >= 2*self.resolution:
                print("add node: " + str(vertex))
                for i, d in enumerate(delta):
                    delta[i] = d * scale
                additional_nodes[next_n] = {"vertex": vertex, "next": next_n, "x": self.mesh.vertex_attribute(vertex, "x") + delta[0], "y": self.mesh.vertex_attribute(vertex, "y") + delta[1], "z": self.mesh.vertex_attribute(vertex, "z") + delta[2]}
        
        # check if x_limit is a list
        if not isinstance(x_limit, list):
            x_limit = [x_limit for _ in range(z_limit)]
        self.insert_nodes(additional_nodes, z_limit, x_limit)
    
    def insert_nodes(self, additional_nodes, z_limit, x_limit):
        # adding additional nodes to the mesh
        counter = 0
        i_counter = [0 for _ in range(z_limit)]
        new_mesh = Mesh()
        vertices = list(self.mesh.vertices())
        for node in vertices:
            x, y, z, i, j = self.mesh.vertex_attributes(node, ["x", "y", "z", "i", "j"])
            if node in additional_nodes.keys():
                # self.mesh.add_vertex(x=additional_nodes[node]["x"], y=additional_nodes[node]["y"], z=additional_nodes[node]["z"], i="add", j="add", glob_id=node+counter, x_disp=0, z_disp=0)  
                new_mesh.add_vertex(x=additional_nodes[node]["x"], y=additional_nodes[node]["y"], z=additional_nodes[node]["z"], i=i + i_counter[j], j=j, glob_id=node+counter, x_disp=0, z_disp=0)
                counter += 1
                i_counter[j] += 1
            # increasing counter by added nodes
            # self.mesh.vertex_attribute(node, "glob_id", value=node+counter)
            new_mesh.add_vertex(x = x, y = y, z = z, i=i+i_counter[j], j=j, glob_id=node+counter, x_disp=0, z_disp=0)
        
        for row in range(z_limit - 1):
            for col in range(x_limit[row] + i_counter[row] - 1):
                a = col*z_limit + row
                b = a + z_limit
                c = b + 1
                d = a + 1
                new_mesh.add_face([a, b, c, d])
        #self.mesh = new_mesh
        #self.mesh.attributes["name"] = "wall"
        self.z_size = z_limit
        self.x_size = [x_limit[row] + i_counter[row] for row in range(z_limit)]


    def shift_nodes(self):
        # shifting nodes so distance is always resolution
        for vertex in self.mesh.vertices():
            # finding x-neighbor
            neighbors = self.mesh.vertex_neighbors(vertex)
            x, y, z = self.mesh.vertex_coordinates(vertex)
            x_n = [self.mesh.vertex_coordinates(n)[0] for n in neighbors]
            # check if there is a next node in x direction
            if len(x_n) < 4:
                x_dir = False
                for neighbors_x in x_n:
                    if neighbors_x - x > 0.02:
                        x_dir = True
                if not x_dir:
                    # print(vertex, x_n)
                    continue
            next_n = neighbors[x_n.index(max(x_n))]
            # calculating distance between nodes
            x_n, y_n, z_n = self.mesh.vertex_coordinates(next_n)
            distance = math.sqrt((x-x_n)**2 + (y-y_n)**2 + (z-z_n)**2)
            scale = self.resolution/distance
            delta = [x_n - x, y_n - y, z_n - z]
            if not (self.resolution-0.01 < distance < self.resolution+0.01) and distance < 2*self.resolution:
                for i, d in enumerate(delta):
                    delta[i] = d * (scale - 1)
                # shifting node
                self.mesh.vertex_attributes(next_n, ["x", "y", "z"], values=[self.mesh.vertex_attribute(next_n, "x") + delta[0], self.mesh.vertex_attribute(next_n, "y") + delta[1], self.mesh.vertex_attribute(next_n, "z") + delta[2]])
        
    def distances(self):
        for vertex in self.mesh.vertices():
            faces = self.mesh.vertex_faces(vertex)
            neighbors = self.mesh.vertex_neighbors(vertex)
            dist = []
            x, y, z = self.mesh.vertex_coordinates(vertex)
            for n in neighbors:
                x_n, y_n, z_n = self.mesh.vertex_coordinates(n)
                dist.append(math.sqrt((x-x_n)**2 + (y-y_n)**2 + (z-z_n)**2))
            self.distances[vertex] = {"faces": faces, "neighbors": neighbors, "dist": dist}
        return self.distances
