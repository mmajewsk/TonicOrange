import orbslam2
from collections import deque
from basic_usage import read_folder_as_tuples
import json
from matplotlib import pyplot as plt
import numpy as np
import time
import pathlib
from mapper import Pose3Dto2D, transform
from slam_map import OsmapData

class TonicMock:
    def __init__(self, images_path, start=0, stop=None):
        self.images_path = images_path
        self.start = start
        self.data = read_folder_as_tuples(self.images_path)
        self.stop = stop
        if self.stop == None:
            self.stop = len(self.data)-1

    def image_now(self):
        for data in self.data[self.start + 1:self.stop]:
            yield data

class LocalisatorMock:
    def __init__(self, vocab_path, settings_path):
        self.slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)

    def initialise(self, initialising_data, my_osmap):
        self.slam.set_use_viewer(True)
        self.slam.initialize()
        self.slam.osmap_init()
        self.slam.activate_localisation_only()
        self.slam.process_image_mono(initialising_data[0], initialising_data[1])
        map_load_path = my_osmap.map_path/(my_osmap.map_name+".yaml")
        self.slam.map_load(str(map_load_path), False, False)


    def localise(self, data):
        img, ts = data
        self.slam.process_image_mono(img, ts)
        return self.slam.get_current_pose()

class NavigatorMock:
    def __init__(self, checkpoints: deque, distance_threshold=None):
        """

        :param checkpoints: list of 3D poses we want to go through
        """
        self.distance_threshold = distance_threshold
        self.checkpoints = checkpoints
        self.direction_pose = None
        self.finish = False

    @staticmethod
    def from_filenames(checkpoints_filepath, orb_slam_map):
        with open(checkpoints_filepath, 'r') as f:
            filenames_list = f.read().splitlines()
        with open(orb_slam_map.map_path/'assoc.json','r') as f:
            assoc_dict = json.load(f)
        assoc_filenames = {filename:id for id, timestamp, filename in assoc_dict['keyframes']}
        for checkpoint_filename in filenames_list:
            if checkpoint_filename not in assoc_filenames:
                raise ValueError("checkpoint_filename was not found in keyframes")
        id_and_pose = {id: pose for id, pose in my_osmap.id_and_pose()}
        checkpoints = deque()
        for checkpoint_filename in filenames_list:
            checkpoint_keyframe_id = assoc_filenames[checkpoint_filename]
            pose = id_and_pose[checkpoint_keyframe_id]
            checkpoints.append(pose)
        return NavigatorMock(checkpoints)

    def set_smart_threshold(self, fraction=0.1):
        distances = []
        tmp_deq = self.checkpoints.copy()
        prev = tmp_deq.popleft()
        for checkpoint in tmp_deq:
            distances.append(self.distance(prev, checkpoint))
            prev = checkpoint
        mean_checkpoint_distance = np.mean(distances)
        self.distance_threshold = fraction * mean_checkpoint_distance


    def go(self):
        while not self.finish:
            if self.direction_pose is None:
                self.direction_pose = self.checkpoints[0]
            yield self.direction_pose

    def refresh_direction(self, current_position):
        if current_position is None:
            pass
        elif self.distance(current_position, self.direction_pose) < self.distance_threshold:
            if len(self.checkpoints) == 0:
                self.finish = True
            else:
                self.direction_pose = self.checkpoints.popleft()

    def distance(self, pose_a, pose_b):
        return np.linalg.norm(pose_a[:2,3]- pose_b[:2, 3])

class Transform3Dto2D:
    def __init__(self, my_osmap):
        self.pose_transformer = Pose3Dto2D(my_osmap.poses_reshaped())

    def transform(self, pose):
        new_pose = self.pose_transformer.transform.dot(pose)
        new_angles = transform.Rotation.from_dcm(new_pose[:3, :3]).as_euler('xyz')
        position = new_pose[:2, 3]
        # !!! new_angles[2] + np.pi !!!!
        return position, new_angles[2]

class DriverMock:
    def __init__(self):
        self.tmpdumpfilepath = '/home/mwm/repositories/TonicOrange/assets/tmp/arrows.csv'

    def drive(self, current_pose2D, destination2D):
        if current_pose2D is not None:
            with open(self.tmpdumpfilepath, 'a') as f:
                write_line = "{} {} {} {} {} {}".format(*current_pose2D[0], current_pose2D[1], *destination2D[0], current_pose2D[1])
                f.write(write_line+"\n")
        print("Current: {},   Destination:{}".format(current_pose2D, destination2D))

if __name__=="__main__":
    vocab_path = "/home/mwm/repositories/slam_dunk/builds/data/ORBvoc.txt"
    settings_path = "/home/mwm/repositories/os2py_applied/TUM-MINE-wide.yaml" #"./assets/settings/TUM-MINE-wide.yaml "
    images_path = "/home/mwm/Desktop/datadumps/01-07-19-drive-v1_22/"
    checkpoints_file = 'assets/checkpoints/checkpoints1.txt'
    osmap_path = './assets/maps/testmap1/initial_tests.yaml'
    "--start 250 --end 500"
    tonic = TonicMock(images_path,start=210)
    image_iterator = tonic.image_now()
    initialising_data = next(image_iterator)
    my_osmap = OsmapData.from_map_path(osmap_path)
    # this below is needed to calculate the 2D surface
    transformator = Transform3Dto2D(my_osmap)
    navigator = NavigatorMock.from_filenames(checkpoints_file, orb_slam_map=my_osmap)
    navigator.set_smart_threshold()
    localisator = LocalisatorMock(vocab_path=vocab_path,settings_path=settings_path)
    localisator.initialise(initialising_data, my_osmap)
    driver = DriverMock()
    for destination in navigator.go():
        data = next(image_iterator)
        current_im, ts = data
        print(ts)
        pose3D = localisator.localise(data)
        if pose3D is not None:
            current_pose2D = transformator.transform(pose3D)
        else:
            current_pose2D = None
        destination2D = transformator.transform(destination)
        driver.drive(current_pose2D, destination2D)
        navigator.refresh_direction(pose3D)
