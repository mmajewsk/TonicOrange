from tonic import Tonic
from queue import Queue
import numpy as np
from simple_pid import PID
import time
import logging
from common.pose import calculate_desired_angle

logger = logging.getLogger(__name__)

class Action:
    def __init__(self, action_type=None, t=0.0):
        self.t = t
        self.action_type = action_type

class State:
    def __init__(self):
        pass

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

    def assign_state(self, current_pose2D, destination2D):
        if current_pose2D is None:
            logger.debug("No pose!!!")
            self.state['no_pose'].react()
            self.no_pose_counter += 1
            self.resting = True
            self.how_to_turn = None
            self.turning = False
            self.action_time_steps = 15
        else:
            self.no_pose_counter = 0

    def do_action(self):
        pass

    def drive(self, current_pose2D, destination2D):
        time.sleep(0.05)
        self.assign_state()
        self.do_action()

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
