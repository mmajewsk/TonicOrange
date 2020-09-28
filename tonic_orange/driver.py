
from tonic import Tonic
from queue import Queue
import numpy as np
from simple_pid import PID
import time
import logging
from common.pose import calculate_desired_angle

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

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
        self.turning_motor_speed = 17
        self.no_pose_counter = 0
        self.now_resting = True
        self.current_steering_data = None
        self.current_move_type = None
        self.a_t = 0
        self.moving_keys = ['orienting', 'approaching', 'turning_left',  'turning_right', 'resting']
        self.non_moving_keys = ['no_pose', ' at_checkpoint']
        self.states = {}
        for k in self.moving_keys+self.non_moving_keys:
            self.states[k] = False

    def check_orientation(self, current_pose2D, destination2D):
        desired_angle = calculate_desired_angle(current_pose2D, destination2D)
        direction_error_marigin = 15
        if abs(desired_angle) < np.deg2rad(direction_error_marigin):
            return True
        else:
            return False

    def reset_states(self):
        for k in self.states:
            self.states[k] = False

    def assign_state(self, current_pose2D, destination2D):
        self.reset_states()
        logger.debug("a_t {}".format(self.a_t))
        if self.a_t == 0:
            if self.now_resting:
                logger.debug("Now rest")
                self.now_resting = False
                self.states['resting'] = True
                return
            else:
                self.now_resting = True

        if current_pose2D is None:
            logger.debug("No pose!!!")
            self.states['orienting'] = True
            self.states['resting'] = False

        if not self.navigator_queue.empty():
            message = self.navigator_queue.get()
            logger.debug("Got navigator message: {}".format(message))
            #@TODO turn off after one tick
            self.states['at_checkpoint'] = True

        if current_pose2D is not None:
            oriented = self.check_orientation(current_pose2D, destination2D)
            if oriented:
                self.states['approaching'] = True
                logger.debug("Approaching: {}".format(self.a_t))
            else:
                desired_angle = calculate_desired_angle(current_pose2D, destination2D)
                turning_type = 'turning_left' if desired_angle >= 0 else 'turning_right'
                self.states[turning_type] = True
                logger.debug("Turning {}: {}".format(turning_type, self.a_t))


    def do_steering(self, move_type):
        data = {
            'resting':
            (
                25,
                dict(left=0, right=0)
            ),
            'turning_left':
            (
                4,
                dict(left=self.base_speed, right=self.turning_motor_speed)
            ),
            'orienting':
            (
                4,
                dict(left=self.base_speed, right=self.turning_motor_speed)
            ),

            'turning_right':
            (
                4,
                dict(left=self.turning_motor_speed, right=self.base_speed)
            ),
            'approaching':
            (
                4,
                dict(left=15, right=15)
            ),
        }[move_type]
        self.a_t = data[0]
        steering_data = data[1]
        self.current_steering_data = data[1]
        self.current_move_type = move_type
        if steering_data is not None:
            self.tonic.steer_motors(steering_data)

    def do_action(self):
        time.sleep(0.05)
        if self.a_t > 0:
            self.a_t -= 1
            logger.debug("Doing {} == {} T={}".format(self.current_move_type, self.current_steering_data, self.a_t))
            self.tonic.steer_motors(self.current_steering_data)
        else:
            moving_dict = {k:self.states[k] for k in self.moving_keys}
            assert sum(moving_dict.values()) == 1, str(moving_dict)
            for k in self.moving_keys:
                if self.states[k]:
                    self.do_steering(k)


    def drive(self, current_pose2D, destination2D):
        time.sleep(0.05)
        self.assign_state(current_pose2D, destination2D)
        self.do_action()
