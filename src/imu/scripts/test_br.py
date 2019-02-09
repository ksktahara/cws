#!/usr/bin/env python

import math

import rospy
import tf
from sensor_msgs.msg import Joy

RATE = 30

class Broadcaster(object):

    SPEED = math.radians(90) # rad. / s

    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.trans = (1.0, 0.0, 0.0)
        self.rot = [0.0, 0.0, 0.0]
        self.speed = [0.0, 0.0, 0.0]

        self.sub = rospy.Subscriber("joy", Joy, self.callback)

    def callback(self, msg):
        ly = msg.axes[1]
        a = msg.buttons[0]
        b = msg.buttons[1]
        x = msg.buttons[2]
        st = msg.buttons[7]

        if st:
            self.rot = [0.0, 0.0, 0.0]
            self.speed = [0.0, 0.0, 0.0]
            return

        for i, btn in enumerate((b, a, x)): # roll pitch yaw
            if not btn:
                self.speed[i] = 0.0
            else:
                self.speed[i] = ly * self.SPEED / RATE

        rospy.logdebug("r: " + " ".join(["%2.2f" % math.degrees(r) for r in self.rot]))
        rospy.logdebug("s: " + " ".join(["%2.2f" % math.degrees(s) for s in self.speed]))

    def normalize(self, angle):
        while angle >= math.pi:
            angle -= math.pi * 2
        while angle < - math.pi:
            angle += math.pi * 2
        return angle

    def update_rot(self):
        for i in range(len(self.rot)):
            self.rot[i] += self.speed[i]
            self.rot[i] = self.normalize(self.rot[i])

    def broadcast(self):
        rot = tf.transformations.quaternion_from_euler(*self.rot)

        self.br.sendTransform(self.trans, rot,
                rospy.Time.now(), "imu", "map")

    def loop(self):
        self.update_rot()
        self.broadcast()

def main():
    rospy.init_node('test_br')

    br = Broadcaster()
    rate = rospy.Rate(RATE)

    try:
        while not rospy.is_shutdown():
            br.loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__': main()

