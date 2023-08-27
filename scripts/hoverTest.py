"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time
from threading import Thread
from threading import Timer
import random

import cflib
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)


        print('Connecting to %s' % link_uri)
        self.is_connected = True

        # change the parameters0, 
        self._param_check_list = []
        self._param_groups = []

    # def _connected(self, link_uri):
    #     """ This callback is called form the Crazyflie API when a Crazyflie
    #     has been connected and the TOCs have been downloaded."""

    #     # Start a separate thread to do the motor test.
    #     # Do not hijack the calling thread!
    #     Thread(target=self._ramp_motors).start()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        ###########################################################

        # LOG
        # The definition of the logconfig can be made before connecting
        # self._lg_stab = LogConfig(name='packet', period_in_ms=10)
        # self._lg_stab.add_variable('crtp.COMMANDER_FLAG', 'uint8_t')
        # self._lg_stab.add_variable('stabilizer.pitch', 'float')
        # self._lg_stab.add_variable('stabilizer.yaw', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        # try:
        #     self._cf.log.add_config(self._lg_stab)
        #     # This callback will receive the data
        #     self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
        #     # This callback will be called on errors
        #     self._lg_stab.error_cb.add_callback(self._stab_log_error)
        #     # Start the logging
        #     self._lg_stab.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Stabilizer log config, bad configuration.')

        ###########################################################

        # PARAM
        # self._cf.param.add_update_callback(group='motorPowerSet', name='enable',
        #                                    cb=self._a_propTest_callback)
        # self._cf.param.set_value('motorPowerSet.enable',
        #                              '{:d}'.format(1))

        # thv = 2000
        # self._cf.param.set_value('motorPowerSet.m4',
        #                              '{:d}'.format(8000))
        # time.sleep(5)

        # self._cf.param.set_value('motorPowerSet.m4',
        #                              '{:d}'.format(0))
        # self._cf.param.set_value('motorPowerSet.enable',
        #                              '{:d}'.format(0))
        ###########################################################

        # LOG
        # self._lg_stab = LogConfig(name='motor', period_in_ms=100)
        # self._lg_stab.add_variable('motor.m1', 'uint16_t')
        # self._lg_stab.add_variable('motor.m2', 'uint16_t')
        # self._lg_stab.add_variable('motor.m3', 'uint16_t')
        # self._lg_stab.add_variable('motor.m4', 'uint16_t')
        # try:
        #     self._cf.log.add_config(self._lg_stab)
        #     # This callback will receive the data
        #     self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
        #     # This callback will be called on errors
        #     self._lg_stab.error_cb.add_callback(self._stab_log_error)
        #     # Start the logging
        #     self._lg_stab.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Stabilizer log config, bad configuration.')

        # RAMP
        # Thread(target=self._ramp_motors).start()
        self._ramp_motors()

        # Start a timer to disconnect in 1s
        t = Timer(1, self._cf.close_link)
        t.start()

    def _a_propTest_callback(self, name, value):
        """Callback for pid_attitude.pitch_kd"""
        print('Readback: {0}={1}'.format(name, value))

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data), flush=True)
    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 100
        thrust_dstep = 10
        thrust = 3000
        pitch = 0
        roll = 0
        yawrate = 0
        start_height = 0.3
        target_height = 0.8 # the distance is not accurate, 1.2 => 1.5m

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        print("up")
        cnt = 0
        while cnt < 20:
            self._cf.commander.send_hover_setpoint(0, 0, 0, start_height)
            # print(h)
            cnt = cnt + 1
            time.sleep(0.1)
        cnt = 0

        cnt = 0
        while cnt < 20:
            self._cf.commander.send_hover_setpoint(0, 0, 0, target_height)
            cnt = cnt + 1
            time.sleep(0.1)

        # cnt = 0
        # while cnt < 40:
        #     self._cf.commander.send_hover_setpoint(0, 0, 0, target_height) # (forward, left)
        #     cnt = cnt + 1
        #     time.sleep(0.1)

        print("down")
        cnt = 0
        while cnt < 10:
            h = 0.2
            # print(h)
            self._cf.commander.send_hover_setpoint(0, 0, 0, h)
            cnt += 1
            time.sleep(0.1)

        # while thrust >= 1000 and thrust < 10000:
        #     self._cf.commander.send_setpoint(0, 0, 0, thrust)
        #     time.sleep(0.1)
        #     thrust += thrust_step * thrust_mult

        # cnt = 0
        # while cnt < 1:
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     cnt = cnt + 1
        #     time.sleep(0.1)
        # print("cnt")
        # while thrust >= 37000:
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     time.sleep(0.1)
        #     thrust -= thrust_dstep * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print('end', flush=True)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(3)
        # self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    le = MotorRampExample(uri)
    
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    # while le.is_connected:
    #     time.sleep(1)