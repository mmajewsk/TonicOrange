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
from simple_pid import PID
from tonic import Tonic

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

class Localisator:
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

class Navigator:
    def __init__(self, checkpoints: deque, distance_threshold=None):
        """

        :param checkpoints: list of 3D poses we want to go through
        """
        self.distance_threshold = distance_threshold
        self.checkpoints = checkpoints
        self.target_pose = None
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
        return Navigator(checkpoints)

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
            if self.target_pose is None:
                self.target_pose = self.checkpoints[0]
            yield self.target_pose

    def refresh_direction(self, current_position):
        if current_position is None:
            pass
        elif self.distance(current_position, self.target_pose) < self.distance_threshold:
            if len(self.checkpoints) == 0:
                self.finish = True
            else:
                self.target_pose = self.checkpoints.popleft()

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
    def __init__(self, tonic):
        self.tmpdumpfilepath = '/home/mwm/repositories/TonicOrange/assets/tmp/arrows.csv'

    def drive(self, current_pose2D, destination2D):
        if current_pose2D is not None:
            with open(self.tmpdumpfilepath, 'a') as f:
                write_line = "{} {} {} {} {} {}".format(*current_pose2D[0], current_pose2D[1], *destination2D[0], current_pose2D[1])
                f.write(write_line+"\n")
        print("Current: {},   Destination:{}".format(current_pose2D, destination2D))



class Driver:
    def __init__(self, tonic: Tonic):
        self.tonic = tonic
        self.pid = PID(5,1.0,1.0,setpoint=0)
        self.pid.output_limits = [0,10]
        self.base_speed = 15

    def calculate_steering(self, current_pose2D, destination2D):
        (x1, y1), phi1 = current_pose2D
        (x2, y2), phi2 = destination2D
        phi1_d = phi1 + np.pi
        a = np.abs(x1 - x2)
        b = np.abs(y1 - y2)
        p1_p2_angle = np.arctan(b / a)
        output = self.pid(p1_p2_angle-phi1_d)
        return output

    def steer(self, output):
        left_motor = self.base_speed + output
        right_motor = self.base_speed - output
        self.tonic.steer_motors(dict(left=left_motor, right=right_motor))

    def drive(self, current_pose2D, destination2D):
        if current_pose2D is not None:
            output = self.calculate_steering(current_pose2D, destination2D)
            self.steer(output)
        else:
            self.try_orient()

    def try_orient(self):
        pass

if __name__=="__main__":
    vocab_path = "/home/mwm/repositories/slam_dunk/builds/data/ORBvoc.txt"
    settings_path = "/home/mwm/repositories/os2py_applied/TUM-MINE-wide.yaml" #"./assets/settings/TUM-MINE-wide.yaml "
    images_path = "/home/mwm/Desktop/datadumps/01-07-19-drive-v1_22/"
    checkpoints_file = 'assets/checkpoints/checkpoints1.txt'
    osmap_path = './assets/maps/testmap1/initial_tests.yaml'
    tonic_settings = '/home/mwm/repositories/Tonic/src/pc/settings.yaml'
    "--start 250 --end 500"
    mock = True
    if mock:
        tonic = TonicMock(images_path,start=210)
    else:
        tonic = Tonic(tonic_settings)
    image_iterator = tonic.image_now()
    initialising_data = next(image_iterator)
    my_osmap = OsmapData.from_map_path(osmap_path)
    # this below is needed to calculate the 2D surface
    transformator = Transform3Dto2D(my_osmap)
    navigator = Navigator.from_filenames(checkpoints_file, orb_slam_map=my_osmap)
    navigator.set_smart_threshold()
    localisator = Localisator(vocab_path=vocab_path,settings_path=settings_path)
    localisator.initialise(initialising_data, my_osmap)
    if mock:
        driver = DriverMock(tonic)
    else:
        driver = Driver(tonic)
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
