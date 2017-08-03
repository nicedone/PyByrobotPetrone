#!/usr/bin/env python
import rospy
from collections import defaultdict
from std_msgs.msg import Float32, String, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from petrone import Petrone, PETRONE_FLIGHT_EVENT


class RosPetroneNode:
    def __init__(self):
        self.petrone = Petrone()

        # subscriber
        self.sub_flight = rospy.Publisher('cmd_fly', Int8, self.cb_fly, queue_size=1)
        self.sub_cmd = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd, queue_size=1)

        # publisher
        self.pub_battery = rospy.Publisher('battery', Float32, queue_size=1)
        self.pub_status_flight = rospy.Publisher('status/flight', Int8, queue_size=1)
        self.pub_status_system = rospy.Publisher('status/system', Int8, queue_size=1)
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1)

        # cache
        self.is_disconnected = True
        self.last_values = defaultdict(lambda: 0)

    def connect(self, hci, address):
        address = self.petrone.connect(hci, address)
        rospy.loginfo('petrone connected addr={}'.format(address))
        self.is_disconnected = False

    def disconnect(self):
        self.is_disconnected = True
        self.petrone.disconnect()

    def cb_cmd(self, data):
        self.petrone.control(int(data.linear.y), int(data.linear.x), int(data.angular.z), int(data.linear.z))

    def cb_fly(self, data):
        if data == PETRONE_FLIGHT_EVENT.Ready:
            pass
        elif data == PETRONE_FLIGHT_EVENT.TakeOff:
            self.petrone.cmd_takeoff()
        elif data == PETRONE_FLIGHT_EVENT.Stop:
            self.petrone.cmd_stop()
        elif data == PETRONE_FLIGHT_EVENT.Landing:
            self.petrone.cmd_landing()
        elif data == PETRONE_FLIGHT_EVENT.Accident:
            self.petrone.cmd_accident()

    def run(self):
        rospy.sleep(2.0)
        while not self.is_disconnected:
            imu = self.petrone.get_imu()
            # TODO
            rospy.sleep(1.0)

            state = self.petrone.get_state()
            if self.last_values['battery'] != state['battery']:
                self.last_values['battery'] = state['battery']
                self.pub_battery.publish(state['battery'])

            # TODO

            rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('petrone_ros_node', anonymous=True)

    bluetooth_hci = int(rospy.get_param('~hci', ''))
    device_address = rospy.get_param('~address', '')

    rospy.loginfo('init petrone node+')
    node = RosPetroneNode()
    node.connect(bluetooth_hci, device_address)

    rospy.loginfo('start petrone node+')
    node.run()

    node.disconnect()
    rospy.loginfo('shutdown petrone node.')
