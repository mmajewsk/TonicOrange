import sys
import orbslam2
from collections import deque
from queue import Queue
from basic_usage import read_folder_as_tuples
import cv2
import json
from matplotlib import pyplot as plt
import numpy as np
import time
import pathlib
from mapper import Pose3Dto2D, transform
from slam_map import OsmapData
from simple_pid import PID
from tonic import Tonic
import logging
from logging.handlers import RotatingFileHandler
logger = logging.getLogger('TonicOrange')
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
fh = RotatingFileHandler('direction_finding.log', mode='a', maxBytes=5*1024*1024,
                                 backupCount=2, encoding=None, delay=0)
fh.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s][%(funcName)s]%(message)s')
fh.setFormatter(formatter)
logger.addHandler(fh)

def calculate_desired_angle(current_pose2D, destination2D):
        (x1, y1), phi1 = current_pose2D
        (x2, y2), phi2 = destination2D
        phi1_d = phi1 + np.pi
        a = np.abs(x1 - x2)
        b = np.abs(y1 - y2)
        p1_p2_angle = np.arctan(b / a)
        return p1_p2_angle


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
        self.prevtime = None

    def initialise(self, initialising_data, my_osmap):
        self.slam.set_use_viewer(True)
        self.slam.initialize()
        self.slam.osmap_init()
        self.slam.activate_localisation_only()
        print(initialising_data)
        self.slam.process_image_mono(initialising_data[0], initialising_data[1])
        map_load_path = my_osmap.map_path/(my_osmap.map_name+".yaml")
        self.slam.map_load(str(map_load_path), False, False)
        logger.debug("map loaded")


    def localise(self, data):
        img, ts = data
        data = tonic.image_now()
        if self.prevtime is not None:
                print(ts, self.prevtime, ts-self.prevtime)
        start_time = timeit.default_timer()
        self.slam.process_image_mono(img, ts)
        elapsed = timeit.default_timer() - start_time
        print("process {}".format(elapsed))
        pose = self.slam.get_current_pose()
        self.prevtime = ts
        return pose

class Navigator:
    def __init__(self, checkpoints: deque, distance_threshold=None):
        """

        :param checkpoints: list of 3D poses we want to go through
        """
        self.distance_threshold = distance_threshold
        self.checkpoints = checkpoints
        self.target_pose = None
        self.finish = False
        self._signals = []

    @staticmethod
    def from_filenames(checkpoints_filepath, orb_slam_map):
        with open(checkpoints_filepath, 'r') as f:
            filenames_list = f.read().splitlines()
        with open(orb_slam_map.map_path/'assoc.json','r') as f:
            assoc_dict = json.load(f)
        assoc_filenames = {filename:id for id, timestamp, filename in assoc_dict['keyframes']}
        for checkpoint_filename in filenames_list:
            if checkpoint_filename not in assoc_filenames:
                raise ValueError("checkpoint_filename {} was not found in keyframes".format(checkpoint_filename))
        id_and_pose = {id: pose for id, pose in my_osmap.id_and_pose()}
        checkpoints = deque()
        for checkpoint_filename in filenames_list:
            checkpoint_keyframe_id = assoc_filenames[checkpoint_filename]
            pose = id_and_pose[checkpoint_keyframe_id]
            checkpoints.append(pose)
        return Navigator(checkpoints)

    def set_smart_threshold(self, fraction=0.2):
        distances = []
        tmp_deq = self.checkpoints.copy()
        prev = tmp_deq.popleft()
        for checkpoint in tmp_deq:
            distances.append(self.distance(prev, checkpoint))
            prev = checkpoint
        mean_checkpoint_distance = np.mean(distances)
        self.distance_threshold = fraction * mean_checkpoint_distance

    def register_signal(self, queue):
        self._signals.append(queue)

    def signals(self, message):
        for q in self._signals:
            q.put(message)

    def go(self):
        while not self.finish:
            if self.target_pose is None:
                self.target_pose = self.checkpoints[0]
            yield self.target_pose

    def refresh_direction(self, current_position):
        if current_position is None:
            pass
        else:
            distance = self.distance(current_position, self.target_pose)
            if distance < self.distance_threshold:
                logger.debug("Witihin range (distance: {}) of checkpoint of {}".format(distance, self.target_pose))
                if len(self.checkpoints) == 0:
                    self.finish = True
                    self.signals(("finish", time.time()))
                else:
                    self.target_pose = self.checkpoints.popleft()
                    self.signals(("checkpoint_reached", time.time()))
                    logger.debug("Left checkpoints: {}. Changing to checkpoint {}".format(len(self.checkpoints), self.target_pose))

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
        self.pid.output_limits = [0,5]
        self.base_speed = 12
        self.navigator_queue = Queue()

    def calculate_steering(self, current_pose2D, destination2D):
        desired_angle = calculate_desired_angle(current_pose2D, destination2D)
        (x1, y1), phi1 = current_pose2D
        (x2, y2), phi2 = destination2D
        phi1_d = phi1 + np.pi
        output = self.pid(desired_angle-phi1_d)
        return output

    def steer(self, output):
        left_motor = self.base_speed + output
        right_motor = self.base_speed - output
        data = dict(left=left_motor, right=right_motor)
        self.tonic.steer_motors(data)
        logger.debug(data)

    def drive(self, current_pose2D, destination2D):
        if current_pose2D is not None:
            output = self.calculate_steering(current_pose2D, destination2D)
            self.steer(output)
        else:
            self.try_orient()

    def try_orient(self):
        pass

class DriverDumb:
    def __init__(self, tonic: Tonic):
        self.tonic = tonic
        self.base_speed = 6
        self.turning = False
        self.approaching = False
        self.resting = False
        self.action_time_steps = 0
        self.navigator_queue = Queue()
        self.how_to_turn = None
        self.turning_motor_speed = 20
        self.no_pose_counter = 0

    def check_orientation(self, current_pose2D, destination2D):
        desired_angle = calculate_desired_angle(current_pose2D, destination2D)
        direction_error_marigin = 15
        if abs(desired_angle) < np.deg2rad(direction_error_marigin):
            return True
        else:
            return False

    def check_if_continue_action(self):
        self.action_time_steps -= 1
        return self.action_time_steps >= 0

    def turn(self, desired_angle):
        if desired_angle <= 0:
            logger.debug("Turning left")
            data = dict(left=self.base_speed, right=self.turning_motor_speed)
        else:
            logger.debug("Turning right")
            data = dict(left=self.turning_motor_speed, right=self.base_speed)
        logger.debug(data)
        self.tonic.steer_motors(data)

    def go_straight(self):
        logger.debug("Going straight")
        data = dict(left=15, right=15)
        logger.debug(data)
        self.tonic.steer_motors(data)

    def process_resting(self):
        self.resting = self.check_if_continue_action()
        if self.resting:
            data = dict(left=0, right=0)
            self.tonic.steer_motors(data)
            logger.debug("Resting {}".format(self.action_time_steps))

    def drive(self, current_pose2D, destination2D):
        time.sleep(0.05)
        if current_pose2D is None:
            logger.debug("No pose!!!")
            self.no_pose_counter += 1
            self.resting = True
            self.how_to_turn = None
            self.turning = False
            self.action_time_steps = 15
        else:
            self.no_pose_counter = 0
        if self.resting and self.no_pose_counter > 3:
            logger.debug("resting")
            self.process_resting()
            return
        if not self.navigator_queue.empty():
            message = self.navigator_queue.get()
            logger.debug("Got navigator message: {}".format(message))
            #@TODO handle finish message
            self.approaching = False
            self.turning = False
            self.at_checkpoint = True
        if self.turning:
            logger.debug("turning {}".format(self.action_time_steps))
            self.turning = self.check_if_continue_action()
            if self.turning:
                # @TODO make it not to calculate this thing every time but
                # make it calculated once and then repeated as action
                #
                logger.debug("turning more")
                if self.how_to_turn is None:
                     self.how_to_turn = calculate_desired_angle(current_pose2D, destination2D)
                self.turn(self.how_to_turn)
            else:
                self.resting = True
                self.how_to_turn = None
                self.action_time_steps = 15
            return
        if self.approaching:
            logger.debug("approaching")
            self.approaching = self.check_if_continue_action()
            if self.approaching:
                self.go_straight()
            else:
                self.resting = True
                self.action_time_steps = 7
            return
        if current_pose2D is not None:
            oriented = self.check_orientation(current_pose2D, destination2D)
            if oriented:
                self.approaching = True
            else:
                self.turning = True
            self.action_time_steps = 7
        else:
            if self.check_if_continue_action():
                self.no_pose_counter = 0
                self.try_orient()
                self.resting = True

    def try_orient(self):
        logger.debug("Orienting (Turning right)")
        data = dict(left=self.base_speed, right=self.turning_motor_speed)
        logger.debug(data)
        self.tonic.steer_motors(data)



import timeit
if __name__=="__main__":
    vocab_path = "/home/mwm/repositories/slam_dunk/builds/data/ORBvoc.txt"
    settings_path = "/home/mwm/repositories/os2py_applied/TUM-MINE-wide.yaml" #"./assets/settings/TUM-MINE-wide.yaml "
    images_path = "/home/mwm/repositories/Tonic/data_intake4/27_07_2020_home_v0.02"
    checkpoints_file = '/home/mwm/repositories/TonicOrange/assets/checkpoints/checkpoints_home_27_07_2020v2.txt'
    osmap_path = '/home/mwm/repositories/TonicOrange/assets/maps/home_27_07_2020_map1c/initial_tests.yaml'
    tonic_settings = '/home/mwm/repositories/Tonic/src/pc/settings.yaml'
    "--start 250 --end 500"
    mock = False
    if mock:
        tonic = TonicMock(images_path,start=210)
    else:
        tonic = Tonic(tonic_settings)
        tonic.connect_video()
    print("wait for camera to start")
    image = None
    while image is None:
        time.sleep(2)
        initialising_data = tonic.image_now()
        image = initialising_data[0]
    my_osmap = OsmapData.from_map_path(osmap_path)
    # this below is needed to calculate the 2D surface
    transformator = Transform3Dto2D(my_osmap)
    navigator = Navigator.from_filenames(checkpoints_file, orb_slam_map=my_osmap)
    navigator.set_smart_threshold(0.05)
    localisator = Localisator(vocab_path=vocab_path,settings_path=settings_path)
    logger.debug("initialising localisator")
    localisator.initialise(initialising_data, my_osmap)
    #if mock:
    #    driver = DriverMock(tonic)
    #else:
    logger.debug("starting driver")
    driver = DriverDumb(tonic)
    start = time.time()
    start_time = timeit.default_timer()
    print("Start navigating")
    navigator.register_signal(driver.navigator_queue)
    for destination in navigator.go():
        data = tonic.image_now()
        current_im, ts = data
        pose3D = localisator.localise(data)
        if pose3D is not None:
            current_pose2D = transformator.transform(pose3D)
        else:
            current_pose2D = None
        destination2D = transformator.transform(destination)
        driver.drive(current_pose2D, destination2D)
        navigator.refresh_direction(pose3D)
        if time.time() - start > 25:
                localisator.slam.shutdown()
                sys.exit()
