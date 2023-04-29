from roslibpy import Ros, Topic

from compas.geometry import Point, Frame, Quaternion, Translation, distance_point_point, KDTree, angle_vectors
from compas.data import Data

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
