import logging
from ctypes import *

import time
from threading import Lock

from bluepy import btle
from bluepy.blescan import dump_services

from constants import *
from scan import petrone_list


ble_lock = Lock()


def on_thread_lock(original_function):
    def new_function(*args, **kwargs):
        ble_lock.acquire()
        try:
            result = original_function(*args, **kwargs)
        except:
            return
        finally:
            ble_lock.release()
        return result
    return new_function


class Petrone:
    @staticmethod
    def to_1byte(number):
        return c_ubyte(number)

    @staticmethod
    def bytes_to_str(bytelist):
        bytelist = [Petrone.to_1byte(x) if not isinstance(x, c_ubyte) else x for x in bytelist]
        buff = (c_ubyte * len(bytelist))(*bytelist)
        bs = bytearray(buff)
        return str(bs)

    @staticmethod
    def s16_to_int(bytelist):
        if bytelist[1] >= 128:
            return -1 * (~(bytelist[0] + bytelist[1] * 256) + 2**16)
        else:
            return bytelist[0] + bytelist[1] * 256

    def __init__(self, send_cmd=True):
        self.petrone_bt = None
        self.send_cmd = send_cmd

    def connect(self, hci=0, addr=None):
        """
        :param hci: bluetooth device hci (default= 0)
        :param addr: petrone device address. if not provided, automatically select the first petrone.
        """
        if not addr:
            candidates = petrone_list(hci)
            if not candidates:
                raise Exception('No Petrone Found.')
            addr = candidates[0]
            logging.info('Petrone Address={}'.format(addr.addr))
        self.petrone_bt = btle.Peripheral(addr)
        # self.petrone_bt.connect(addr=addr)
        logging.info('Petrone Connected.')
        dump_services(self.petrone_bt)

        return addr

    def disconnect(self):
        self.petrone_bt.disconnect()

    def _get_drone_conf(self):
        return self.petrone_bt.getServiceByUUID(PETRONE_SERVICE.DRONE_SERVICE) \
            .getCharacteristics(PETRONE_CHARACTERISTIC.DRONE_CONF)[0]

    def _get_drone_data(self):
        return self.petrone_bt.getServiceByUUID(PETRONE_SERVICE.DRONE_SERVICE) \
            .getCharacteristics(PETRONE_CHARACTERISTIC.DRONE_DATA)[0]

    def _read_drone_data(self):
        return self._get_drone_data().read()

    @on_thread_lock
    def mode(self, mode):
        bytelist = [
            PETRONE_DATATYPE.Command,
            PETRONE_COMMAND.ModePetrone,
            mode
        ]
        self._get_drone_conf().write(Petrone.bytes_to_str(bytelist))

    @on_thread_lock
    def _flight_cmd(self, cmd):
        if not self.send_cmd:
            return
        bytelist = [
            PETRONE_DATATYPE.Command,
            PETRONE_COMMAND.FlightEvent,
            cmd
        ]
        self._get_drone_conf().write(Petrone.bytes_to_str(bytelist))

    def cmd_ready(self):
        self._flight_cmd(PETRONE_FLIGHT_EVENT.Ready)

    def cmd_takeoff(self):
        self._flight_cmd(PETRONE_FLIGHT_EVENT.TakeOff)

    def cmd_landing(self):
        self._flight_cmd(PETRONE_FLIGHT_EVENT.Landing)

    def cmd_stop(self):
        self._flight_cmd(PETRONE_FLIGHT_EVENT.Stop)

    def cmd_accident(self):
        self._flight_cmd(PETRONE_FLIGHT_EVENT.Accident)

    @on_thread_lock
    def set_trim(self, trim_chg):
        bytelist = [
            PETRONE_DATATYPE.Command,
            PETRONE_COMMAND.Trim,
            trim_chg
        ]
	self._get_drone_conf().write(Petrone.bytes_to_str(bytelist)) 

    @on_thread_lock
    def control(self, roll, pitch, yaw, throttle):
        bytelist = [
            PETRONE_DATATYPE.Control,
            roll, pitch, yaw, throttle
        ]
        self._get_drone_conf().write(Petrone.bytes_to_str(bytelist))

    @on_thread_lock
    def set_led(self, led_mode, led_color_idx):
        if not isinstance(led_color_idx, int):
            led_color_idx = PETRONE_LED_COLOR.get_value(str(led_color_idx))
        bytelist = [
            PETRONE_DATATYPE.LedMode,
            led_mode,
            led_color_idx,
            0x05
        ]
        self._get_drone_conf().write(Petrone.bytes_to_str(bytelist))

    @on_thread_lock
    def _request_info(self, info_key):
        bytelist = [
            PETRONE_DATATYPE.Command,
            PETRONE_COMMAND.Request,
            info_key
        ]
        self._get_drone_conf().write(Petrone.bytes_to_str(bytelist), True)
        b = bytearray(self._get_drone_data().read())
        if len(b) <= 0 or b[0] != info_key:
            return False, None

        return True, b

    def get_imu(self):
        success, val = self._request_info(PETRONE_DATATYPE.ImuRawAndAngle)
        if not success:
            return {}
        return {
            'acc': {
                'x': Petrone.s16_to_int(val[1:]),
                'y': Petrone.s16_to_int(val[3:]),
                'z': Petrone.s16_to_int(val[5:]),
            },
            'gyro': {
                'roll': Petrone.s16_to_int(val[7:]),
                'pitch': Petrone.s16_to_int(val[9:]),
                'yaw': Petrone.s16_to_int(val[11:]),
            },
            'angle': {
                'roll': Petrone.s16_to_int(val[13:]),
                'pitch': Petrone.s16_to_int(val[15:]),
                'yaw': Petrone.s16_to_int(val[17:]),
            }
        }

    def get_state(self):
        success, val = self._request_info(PETRONE_DATATYPE.State)
        if not success:
            return {}
        return {
            'mode': {
                'vehicle': val[1],
                'system': val[2],
                'flight': val[3],
                'drive': val[4]
            },
            'battery': val[-1]
        }


if __name__ == '__main__':
    petrone = Petrone(send_cmd=False)
    petrone.connect(addr=ADDR_PETRONE5815)
    petrone.mode(PETRONE_MODE.FlightFPV)

    petrone.set_led(PETRONE_LED_MODE.ArmHold, PETRONE_LED_COLOR.get_value('Yellow'))

    petrone.cmd_ready()
    time.sleep(0.1)

    petrone.cmd_takeoff()

    for _ in range(200):
        img = petrone.get_imu()
        print(img)
        state = petrone.get_state()
        print(state)
        petrone.control(0, 0, 0, 30)
        time.sleep(0.05)

    # time.sleep(5)
    petrone.cmd_landing()

    petrone.set_led(PETRONE_LED_MODE.ArmHold, PETRONE_LED_COLOR.get_value('Red'))
    petrone.disconnect()
