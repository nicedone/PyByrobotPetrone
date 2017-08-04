#!/usr/bin/env python
import rospy
import json
import time
from collections import defaultdict
from std_msgs.msg import Float32, String, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from petrone import Petrone, PETRONE_FLIGHT_EVENT, PETRONE_MODE, PETRONE_TRIMTYPE

c_gain = 1


class RosPetroneNode:
    def __init__(self):
        self.petrone = Petrone()

        # subscriber
        self.sub_flight = rospy.Subscriber('cmd_fly', Int8, self.cb_fly, queue_size=1)
        self.sub_cmd = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd, queue_size=1)
        self.sub_color = rospy.Subscriber('led_color', String, self.cb_color, queue_size=1)

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
        self.twist_at = 0

    def set_base_thrust(self, base_thrust):
        self.base_thrust = base_thrust

    def connect(self, hci, address):
        address = self.petrone.connect(hci, address)
        rospy.loginfo('petrone connected addr={}'.format(address))
        self.is_disconnected = False

    def disconnect(self):
        self.is_disconnected = True
        self.petrone.disconnect()

    def cb_color(self, data):
        j = json.loads(data.data)
        self.petrone.set_led(j['led_mode'], j['led_color'])

    def cb_cmd(self, data):
        self.twist = data
        self.twist_at = time.time()
        self.petrone.control(int(data.linear.y * c_gain)
                            , int(data.linear.x * c_gain)
                            , int(data.angular.z * c_gain)
                            , int(data.linear.z * c_gain))
        rospy.loginfo('{} {} {} {}'.format(data.linear.y, data.linear.x, data.angular.z, data.linear.z))

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
        rospy.sleep(0.5)
        skip_cnt = 0
        while not self.is_disconnected:
            if skip_cnt % 20 == 0:
                imu = self.petrone.get_imu()

            if skip_cnt % 200 == 0:
                state = self.petrone.get_state()
                if 'battery' in state.keys() and self.last_values['battery'] != state['battery']:
                    self.last_values['battery'] = state['battery']
                    self.pub_battery.publish(state['battery'])
                    rospy.loginfo('battery={}'.format(state['battery']))
                skip_cnt = 1

            skip_cnt += 1
            if time.time() - self.twist_at < 5:
		 self.petrone.control(int(self.twist.linear.y * c_gain)
				    , int(self.twist.linear.x * c_gain * -1)
				    , int(self.twist.angular.z * c_gain * -1)
				    , int(self.twist.linear.z * c_gain))
                 rospy.sleep(0.01) 

    def set_robot_mode(self, mode):
        self.petrone.mode(mode)
        rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('petrone_ros_node', anonymous=True)

    bluetooth_hci = int(rospy.get_param('~hci', '0'))
    device_address = rospy.get_param('~address', '')

    rospy.loginfo('init petrone node+')

    node = RosPetroneNode()
    node.connect(bluetooth_hci, device_address)
    node.set_robot_mode(PETRONE_MODE.FlightFPV)

    base_thrust = int(rospy.get_param('~base_thrust', '0'))
    rospy.loginfo('base thrust adjust={}'.format(base_thrust))
    for _ in range(abs(base_thrust)):
        if base_thrust < 0:
            node.petrone.set_trim(PETRONE_TRIMTYPE.ThrottleDecrease)
        else:
            node.petrone.set_trim(PETRONE_TRIMTYPE.ThrottleIncrease)
    rospy.loginfo('start petrone node+')
    node.run()

    node.disconnect()
    rospy.loginfo('shutdown petrone node.')
