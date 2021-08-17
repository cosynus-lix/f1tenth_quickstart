#!/usr/bin/env python


"""
ref:
https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key
https://stackoverflow.com/questions/24072790/detect-key-press-in-python
"""

import rospy
import std_msgs
from pynput.keyboard import Key, Listener
from ackermann_msgs.msg import AckermannDriveStamped

MAX_LIN_VEL = 3.0  # max vel lin 3 m/s
MAX_ANG_VEL = 0.52 # max vel ang 30 degrees

LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around by keyboard arrows:
            up
   left    down    right

up/down : increase/decrease linear velocity
left/right : increase/decrease angular velocity

shift : force stop

CTRL-C to quit
"""

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

class Agent(object):
    def __init__(self):
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0

    def publish_cmd(self, event=None):
        drive = AckermannDriveStamped()
        drive.drive.speed = self.target_linear_vel
        drive.drive.steering_angle = self.target_angular_vel
        self.drive_pub.publish(drive)

    def on_press(self, key):
        if (key==Key.up):
            self.target_linear_vel = constrain(self.target_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif (key==Key.down):
            self.target_linear_vel = constrain(self.target_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif (key==Key.left):
            self.target_angular_vel = constrain(self.target_angular_vel + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif (key==Key.right):
            self.target_angular_vel = constrain(self.target_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif (key==Key.shift):
            self.target_linear_vel   = 0.0
            self.target_angular_vel  = 0.0
            print(vels(self.target_linear_vel, self.target_angular_vel))
    def on_press_action(key):
        self.on_press(self, key)

    def keep_listenning(self, event=None):
        # Collect events until released
        with Listener(on_press=self.on_press) as listener:
            listener.join()

if __name__ == '__main__':
    rospy.init_node('keyboard_node')
    print(msg)

    agent = Agent()
    # ref: https://roboticsbackend.com/how-to-use-a-ros-timer-in-python-to-publish-data-at-a-fixed-rate/
    rospy.Timer(rospy.Duration(1.0/100.0), agent.keep_listenning)
    rospy.Timer(rospy.Duration(1.0/100.0), agent.publish_cmd)
    rospy.spin()
