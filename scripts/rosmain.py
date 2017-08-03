#!/usr/bin/env python
import rospy
from collections import defaultdict
from std_msgs.msg import Float32, String, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from petrone import Petrone, PETRONE_FLIGHT_EVENT, PETRONE_MODE


class RosPetroneNode:
    def __init__(self):
        self.petrone = Petrone()

        # subscriber
        self.sub_flight = rospy.Subscriber('cmd_fly', Int8, self.cb_fly, queue_size=1)
        self.sub_cmd = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd, queue_size=1)

        # publisher
        self.pub_battery = rospy.Publisher('battery', Float32, queue_size=1)
        self.pub_status_flight = rospy.Publisher('status/flight', Int8, queue_size=1)
        self.pub_status_system = rospy.Publisher('status/system', Int8, queue_size=1)
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1)

        # cache
        self.is_disconnected = True
        self.last_values = defaultdict(lambda: 0)

        # flight parameters
        self.twist = Twist()

    def connect(self, hci, address):
        address = self.petrone.connect(hci, address)
        rospy.loginfo('petrone connected addr={}'.format(address))
        self.is_disconnected = False

    def disconnect(self):
        self.is_disconnected = True
        self.petrone.disconnect()

    def cb_cmd(self, data):
        c_gain = 50
        self.petrone.control(int(data.linear.y * c_gain)
                            , int(data.linear.x * c_gain * -1)
                            , int(data.angular.z * c_gain * -1)
                            , int(data.linear.z * c_gain * -1))
        rospy.sleep(0.04)

    def cb_fly(self, data):
        rospy.loginfo('Command: %s', data)
        if data.data == PETRONE_FLIGHT_EVENT.Ready:
            self.petrone.cmd_ready()
        elif data.data == PETRONE_FLIGHT_EVENT.TakeOff:
            self.petrone.cmd_takeoff()
        elif data.data == PETRONE_FLIGHT_EVENT.Stop:
            self.petrone.cmd_stop()
        elif data.data == PETRONE_FLIGHT_EVENT.Landing:
            self.petrone.cmd_landing()
        elif data.data == PETRONE_FLIGHT_EVENT.Accident:
            self.petrone.cmd_accident()

    def run(self):
        rospy.sleep(2.0)
        while not self.is_disconnected:
            # imu = self.petrone.get_imu()
            # TODO
            # rospy.sleep(1.0)

            # state = self.petrone.get_state()
            # if self.last_values['battery'] != state['battery']:
            #     self.last_values['battery'] = state['battery']
            #     self.pub_battery.publish(state['battery'])

            # TODO

            rospy.sleep(1.0)

    def set_robot_mode(self, mode):
        self.petrone.mode(mode)
        rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('petrone_ros_node', anonymous=True)

    bluetooth_hci = int(rospy.get_param('~hci', ''))
    device_address = rospy.get_param('~address', '')

    rospy.loginfo('init petrone node+')

    node = RosPetroneNode()
    node.connect(bluetooth_hci, device_address)
    node.set_robot_mode(PETRONE_MODE.FlightFPV)

    rospy.loginfo('start petrone node+')
    node.run()

    node.disconnect()
    rospy.loginfo('shutdown petrone node.')
