import numpy as np
import osmap_pb2


class KeyFrame:
    pass

class KeyFrameArray:
    def __init__(self):
        self.array = None

    @staticmethod
    def from_protobuf(self, protobuf_key_frame_array):
        new_array = KeyFrameArray()
        new_array.array = []
        for keyframe in protobuf_key_frame_array:
            self.array.append(KeyFrame.from_protobuf(keyframe))

def read_map_data(main_path, filename, data_type):
    dataCLASS = {
            'mappoints': osmap_pb2.SerializedMappointArray,
            'keyframes': osmap_pb2.SerializedKeyframeArray,
            'features': osmap_pb2.SerializedKeyframeFeaturesArray,
            }
    read_path = main_path/'{}.{}'.format(filename, data_type)
    with open(read_path, 'rb') as a :
            _data = dataCLASS[data_type]()
            _data.ParseFromString(a.read())
    return _data

class OrbSlamMap:
    def __init__(self):
        self.key_frames = None
        self.map_points = None
        self.features = None

    def read_map(self, path, map_name):
        self.key_frames = read_map_data(path, map_name, 'keyframes')
        self.map_points = read_map_data(path, map_name, 'mappoints')
        self.features = read_map_data(path, map_name, 'features')

    def keyframe_to_pose(self, kf):
        pose = list(kf.pose.element)
        reshaped_pose = np.array(pose).reshape(3,4)
        pose_base = np.eye(4)
        pose_base[:3] = reshaped_pose
        return pose_base

    def paint_data(self):
        poses = []
        points = []
        for kf in self.key_frames.keyframe:
            pose_base = self.keyframe_to_pose(kf)
            poses.append(pose_base)
        for p in self.map_points.mappoint:
            position = p.position
            points.append(np.array([position.x, position.y, position.z]))
        return poses, points



