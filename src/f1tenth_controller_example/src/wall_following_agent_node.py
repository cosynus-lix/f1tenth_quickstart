#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

TARGET_DIST_LEFT = 1.2
LOOKING_ANGLE = 0.7
steering_target = 1.0 # bang-bang control
speed = 1.0

KP = 1.0
KI = 0
KD = 0.5

# return distance minimum for the half left and half right
def dist_min(scan_msg):
    amin = scan_msg.angle_min
    amax = scan_msg.angle_max
    step = scan_msg.angle_increment
    values = scan_msg.ranges

    i = 0
    angle = amin
    right = 1000.0
    while i < len(values) and angle <= 0:
        v = values[i]
        if v < right:
            right = v
        angle += step
        i += 1

    left = 1000.0
    while i < len(values):
        v = values[i]
        if v < left:
            left = v
        angle += step
        i += 1

    return (left,right)


def find_angle(scan_msg, target):
    if target < 0:
        target = -target

    amin = scan_msg.angle_min
    amax = scan_msg.angle_max
    step = scan_msg.angle_increment
    values = scan_msg.ranges

    left = -1
    right = -1
    angle = amin
    i = 0
    while i < len(values) and right < 0:
        angle += step
        if angle > -target:
            #print "\tright angle: {}".format(angle-step)
            right = values[i]
        i += 1
    while i < len(values) and left < 0:
        if angle >= target:
            #print "\tleft angle: {}".format(angle)
            left = values[i]
        angle += step
        i += 1
    return (left,right)

class PID:
    def __init__(self, p, i, d):
        self.e = 0
        self.e_prev = 0
        self.cumul = 0
        self.kp = p
        self.ki = i
        self.kd = d

    def step(self, e):
        self.e_prev = self.e
        self.e = e
        self.cumul += e
        deriv = self.e - self.e_prev
        return self.kp * self.e + self.ki * self.cumul + self.kd * deriv

    def __call__(self, e):
        return self.step(e)

class Agent(object):
    def __init__(self):
        # single vehicle lap time racing
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        # head-to-head racing
        '''
        self.drive_pub_ego = rospy.Publisher('/ego_id/drive', AckermannDriveStamped, queue_size=1)
        self.scan_sub_ego = rospy.Subscriber('/ego_id/scan', LaserScan, self.scan_callback_ego, queue_size=1)
        self.drive_pub_opp = rospy.Publisher('/opp_id/drive', AckermannDriveStamped, queue_size=1)
        self.scan_sub_opp = rospy.Subscriber('/opp_id/scan', LaserScan, self.scan_callback_opp, queue_size=1)
        '''

        self.steering = 0
        self.pid = PID(KP,KI,KD)

    def calc_drive_cmd(self, scan_msg):
        print('got scan, now planning...')
        drive = AckermannDriveStamped()
        print "angles: {}".format(find_angle(scan_msg,1.))
        left,right = dist_min(scan_msg)
        drive.drive.speed = speed
        drive.drive.steering_angle = -self.pid(TARGET_DIST_LEFT - left)
        return drive


    # single vehicle lap time racing
    def scan_callback(self, scan_msg):
        self.drive_pub.publish(self.calc_drive_cmd(scan_msg))

    # head-to-head racing
    '''
    def scan_callback_ego(self, scan_msg):
        self.drive_pub_ego.publish(self.calc_drive_cmd(scan_msg))
    def scan_callback_opp(self, scan_msg):
        self.drive_pub_opp.publish(self.calc_drive_cmd(scan_msg))
    '''

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()
