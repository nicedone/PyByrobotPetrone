#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, Float32
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty


from constants import PETRONE_FLIGHT_EVENT


class Controller:
    def __init__(self, joy_topic):
        rospy.loginfo("waiting for petrone")
        # rospy.wait_for_message('battery', Float32)
        # rospy.loginfo("found petrone")

        # fly control publisher
        self.pub_fly = rospy.Publisher('cmd_fly', Int8, queue_size=1)

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self.cb_joy)

    def cb_joy(self, data):
        rospy.loginfo(data)
        for i in range(0, len(data.buttons)):
            if i == 0 and data.buttons[i] == 1: # A
                self.pub_fly.publish(PETRONE_FLIGHT_EVENT.Landing)
                rospy.loginfo("GOT REMOTE KEY => [A: Landing]")
                pass
            if i == 1 and data.buttons[i] == 1: # B
                self.pub_fly.publish(PETRONE_FLIGHT_EVENT.Ready)
                rospy.loginfo("GOT REMOTE KEY => [B: Ready]")
                pass
            if i == 2 and data.buttons[i] == 1: # X
                self.pub_fly.publish(PETRONE_FLIGHT_EVENT.Flip)
                rospy.loginfo("GOT REMOTE KEY => [X: Flip]")
                pass
            if i == 3 and data.buttons[i] == 1: # Y
                self.pub_fly.publish(PETRONE_FLIGHT_EVENT.Flight)
                rospy.loginfo("GOT REMOTE KEY => [Y: Flight]")

if __name__ == '__main__':
    rospy.init_node('petrone_demo_joystick', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(joy_topic)
    rospy.spin()
