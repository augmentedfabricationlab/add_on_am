from roslibpy import Ros, Topic

from compas.geometry import Point, Vector, Circle, Sphere, Cylinder, Plane, Frame, Quaternion, Translation, KDTree, distance_point_point, angle_vectors 
from compas_rhino.conversions import polyline_to_compas, curve_to_compas_polyline
from compas.datastructures import Mesh
from compas.data import Data
import math

class WsSphere(Data):
    def __init__(self, header=None, point=None, ri=None, poses=None):
        self.header = header
        self.point = point
        self.ri = ri
        self.poses = poses

    @property
    def data(self):
        _data = {
            "header" : self.header,
            "point" : self.point.data,
            "ri" : self.ri,
            'poses' : [frame.data for frame in self.poses]
        }
        return _data

    @data.setter
    def data(self, data):
        self.header = data['header']
        self.point = Point.from_data(data['point'])
        self.ri = data['ri']
        self.poses = [Frame.from_data(pose) for pose in data['poses']]

    @classmethod
    def from_data(cls, data):
        sph = cls()
        sph.data = data
        return sph

    @classmethod
    def from_message(cls, message):
        hdr = message["header"]
        pt = message["point"]
        point = Point(pt["x"], pt["y"], pt["z"])
        ri = message["ri"]
        poses = []
        for pose in message["poses"]:
            pos, ori = [pose["position"], pose["orientation"]]
            qt = Quaternion(ori["w"], ori["x"], ori["y"], ori["z"])
            pt = Point(pos["x"],pos["y"], pos["z"])
            poses.append(Frame.from_quaternion(qt, pt))
        return cls(hdr, point, ri, poses)
        

class ReachabilityMap(Data):
    def __init__(self, header=None, spheres=[], resolution=None):
        self.header = header
        self.spheres = spheres
        self.resolution = resolution
        self.points = [point for point, _ in self.spheres_data()]
    
    @property
    def data(self):
        _data = {
            "header" : self.header,
            "spheres" : [sphere.data for sphere in self.spheres],
            "resolution" : self.resolution
        }
        return _data

    @data.setter
    def data(self, data):
        self.header = data['header']
        self.spheres = [WsSphere.from_data(sphere) for sphere in data['spheres']]
        self.resolution = data['resolution']

    @classmethod
    def from_data(cls, data):
        rm = cls()
        rm.data = data
        rm.points = [point for point, _ in rm.spheres_data()]
        return rm

    @classmethod
    def from_message(cls, message):
        hdr = message["header"]
        spheres = []
        for sphere in message["WsSpheres"]:
            spheres.append(WsSphere.from_message(sphere))
        res = message["resolution"]
        rm = cls(hdr, spheres, res)
        rm.points = [point for point, _ in rm.spheres_data()]
        return rm

    def update_from_message(self, message):
        self.header = message["header"]
        self.spheres = []
        for sphere in message["WsSpheres"]:
            self.spheres.append(WsSphere.from_message(sphere))
        self.resolution = message["resolution"]
        self.points = [point for point, _ in self.spheres_data()]
    
    def spheres_data(self):
        for sphere in self.spheres:
            yield sphere.point, sphere.ri
    
    def apply_surface(self, points, center):
        trans = Translation.from_vector(center)
        cloud = Point.transformed_collection(self.points, trans)
        kd = KDTree(cloud)
        reachability_2D = []
        indices = []
        for point in points:
            p, i, dist = kd.nearest_neighbor(point)
            reachability_2D.append(self.spheres[i].ri)
            indices.append(i)
        return reachability_2D, indices
    
    def calc_radius(self):
        d = []
        for i, pointi in enumerate(self.points[:-1]):
            if self.spheres[i].ri <= 10:
                for j, pointj in enumerate(self.points[:-1]):
                    if self.spheres[j].ri <= 10:
                        d.append(distance_point_point(pointi, pointj))
        return max(d)/2
    
    def calc_radius_two(self):
        d = []
        for i, pointi in enumerate(self.points[:-1]):
            d.append(distance_point_point(pointi, self.points[i+1]))
        return max(d)/2
    

class ReachabilityMap2D:
    def __init__(self, spheres=[], indices=[]):
        self.spheres = spheres
        self.indices = indices
    
    @classmethod
    def from3d(cls, map3d, points, center):
        rm = cls([], [])
        trans = Translation.from_vector(center)
        cloud = Point.transformed_collection(map3d.points, trans)
        kd = KDTree(cloud)
        reachability_2D = []
        for i, point in enumerate(points):
            p, k, dist = kd.nearest_neighbor(point)
            rm.spheres.append(map3d.spheres[k].copy())
            rm.spheres[i].point = point
            rm.indices.append(k)
        return rm
    
    def compare_planes(self, normals):
        angles = []
        closest_pose = []
        for i, normal in enumerate(normals):
            temp = []
            for pose in self.spheres[i].poses:
                temp.append(angle_vectors(normal, pose.normal))
            temp_pose = self.spheres[i].poses[temp.index(min(temp))]
            temp_pose.point = self.spheres[i].point
            closest_pose.append(temp_pose)
            angles.append(temp)
        return angles, closest_pose


class Envelope:
    def __init__(self, r_out, h_out, r_in, h_in, z_offset, x=0, y=0, z=0, p=None):
        self.r_out = r_out
        self.h_out = h_out
        self.r_in = r_in
        self.h_in = h_in
        self.z_offset = z_offset
        
        if p != None:
            x = p.X
            y = p.Y
            z = p.Z
        self.x = x
        self.y = y
        self.z = z
        self.mesh_out = None
    
    def draw_mesh(self):
        self.mesh_out = self.create_sphere_cylinder(self.r_out, self.h_out)
        return self.mesh_out
    
    def point_inside(self, x, y, z):
        condition1 = [z > self.h_out+self.z, z < self.z]
        condition2 = {
        "cylinder": (x-self.x)**2 + (y-self.y)**2 < self.r_out**2,
        "upper circle": (x-self.x)**2 + (y-self.y)**2 + (z-self.h_out-self.z)**2 < self.r_out**2,
        "lower circle": (x-self.x)**2 + (y-self.y)**2 + (z-self.z)**2 < self.r_out**2
        }
        outside_condition = ((not (condition1[0] or condition1[1])) and condition2["cylinder"]) or (condition1[0] and condition2["upper circle"]) or (condition1[1] and condition2["lower circle"])
        
        condition1 = [z-self.z_offset > self.h_in+self.z, z-self.z_offset < self.z]
        condition2 = {
        "cylinder": (x-self.x)**2 + (y-self.y)**2 > self.r_in**2,
        "upper circle": (x-self.x)**2 + (y-self.y)**2 + (z-(self.h_in+self.z_offset)-self.z)**2 > self.r_in**2,
        "lower circle": (x-self.x)**2 + (y-self.y)**2 + (z-self.z_offset-self.z)**2 > self.r_in**2
        }
        # inside_condition = ((not (condition1[0] or condition1[1])) and condition2["cylinder"]) or (condition1[0] and condition2["upper circle"]) or (condition1[1] and condition2["lower circle"])
        if not (condition1[0] or condition1[1]):
            inside_condition = condition2["cylinder"]
        elif condition1[0]:
            inside_condition = condition2["upper circle"]
        elif condition1[1]:
            inside_condition = condition2["lower circle"]
        else:
            inside_condition = False
        
        if inside_condition and outside_condition:
            return True
        return False

    def crvs_inside(self, crvs, whole_crv=False):
        crvs_in = []
        crvs_out = []
        for curve in crvs:
            crv = polyline_to_compas(curve.ToPolyline())
            if whole_crv:
                inside = [self.point_inside(p.x, p.y, p.z) for p in crv.divide_by_length(0.05, False)]
                print(inside[0])
                print(inside)
            else:
                ps = crv.point(0.0)
                pe = crv.point(1.0)
                inside = [self.point_inside(ps.x, ps.y, ps.z), self.point_inside(pe.x, pe.y, pe.z)]
            if all(inside):
                crvs_in.append(curve)
            else:
                crvs_out.append(curve)
        return crvs_in, crvs_out
    
    def faces_inside(self, mesh):
        inside = []
        for i, vertex in enumerate(mesh.Vertices):
            if self.point_inside(vertex.X, vertex.Y, vertex.Z):
                inside += mesh.Vertices.GetVertexFaces(i)
        return mesh.DuplicateMesh().Faces.ExtractFaces(inside)


        
    # def create_mesh(self):
    #     reach = self.create_sphere_cylinder(self.r_out, self.h_out)
    #     deadzone = self.create_sphere_cylinder(self.r_in, self.h_in)
    #     deadzone.Translate(0, 0, self.z_offset)
    #     e = rg.Mesh()
    #     e.Append([reach, deadzone])
    #     e.Translate(self.x, self.y, self.z)
    #     return e
    
    def create_sphere_cylinder(self, r, h):
        pill = Mesh.from_shape(Sphere(Point(self.x, self.y, self.z), r))
        to_delete = []
        for face in pill.faces():
            if pill.face_normal(face)[2] > 0:
                to_delete.append(face)
        for face in to_delete:
            pill.delete_face(face)
        
        pill_two = Mesh.from_shape(Sphere(Point(self.x, self.y, self.z + h), r))
        to_delete = []
        for face in pill_two.faces():
            if pill_two.face_normal(face)[2] < 0:
                to_delete.append(face)
        for face in to_delete:
            pill_two.delete_face(face)

        cyl = Mesh.from_shape(Cylinder(Circle(Plane(Point(self.x, self.y, self.z + h/2), Vector(0, 0, 1)), r), h))
        to_delete = []
        for face in cyl.faces():
            if cyl.face_normal(face)[2] != 0:
                to_delete.append(face)
        for face in to_delete:
            cyl.delete_face(face)


        pill.join(pill_two)
        pill.join(cyl)
        return pill



if __name__ == "__main__":
    client = Ros(host="10.152.106.220", port=9090)
    client.run()
    listener = Topic(client, '/reachability_map', 'map_creator/WorkSpace')
    reachabilitymap = ReachabilityMap()
    listener.subscribe(reachabilitymap.update_from_message)

    try:
        while True:
            pass
        else:
            print(len(listener.rm.spheres))
    except KeyboardInterrupt:
        client.terminate()
        print(len(listener.rm.spheres))
