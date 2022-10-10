"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time
from threading import Thread
from threading import Timer
import random

import termios, fcntl, sys, os

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

        # RAMP
        # Thread(target=self._ramp_motors).start()
        self._ramp_motors()

        # Start a timer to disconnect in 15s
        t = Timer(3, self._cf.close_link)
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
        thrust_step = 1500
        thrust_max = 20000
        thrust = 2000
        pitch = 0
        roll = 0
        yawrate = 0
        start_height = 0.2
        defatult_height = 0.8
        target_height = 0.8
        vx = 0
        vy = 0

        # Unlock startup thrust protection
        pk = CRTPPacket()
        pk.set_header(0xF, 2)  # unlock drone
        pk.data = '0'
        self._cf.send_packet(pk)
        self._cf.commander.send_usb_disable()

        fd = sys.stdin.fileno()
        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

        def _constrain(value, vmin, vmax):
            if value > vmax:
                return vmax
            if value < vmin:
                return vmin
            return round(value, 2)


        # take off
        cnt = 0
        while cnt < 10:
            self._cf.commander.send_setpoint(0, 0, 0, thrust_max)
            cnt = cnt + 1
            time.sleep(0.1)
            

        # return
        # goto target height
        cnt = 0
        while cnt < 30:
            self._cf.commander.send_hover_setpoint(0, 0, 0, target_height)
            cnt = cnt + 1
            time.sleep(0.1)


        # enable usb control
        # self._cf.commander.send_usb_enable()
        print("begin control")
        try:
            while 1:
                try:
                    c = sys.stdin.read(1)
                    if c and c != '\x1b':
                        if c == 'w':
                            vx = _constrain(vx + 0.1, -0.4, 0.4)
                        elif c == 's':
                            vx = _constrain(vx - 0.1, -0.4, 0.4)
                        elif c == 'a':
                            vy = _constrain(vy + 0.1, -0.4, 0.4)
                        elif c == 'd':
                            vy = _constrain(vy - 0.1, -0.4, 0.4)
                        elif c == '8':
                            target_height = _constrain(target_height + 0.1, 0.2, 1.7)
                        elif c == '2':
                            target_height = _constrain(target_height - 0.1, 0.2, 1.7)
                        elif c == 'r':
                            vx = 0
                            vy = 0
                        elif c == 'p':
                            self._cf.commander.send_usb_disable()
                            vx = 0
                            vy = 0
                            target_height = defatult_height
                        elif c == 'o':
                            self._cf.commander.send_usb_enable()
                        elif c == 'h':
                            target_height = defatult_height

                        self._cf.commander.send_hover_setpoint(vx, vy, 0, target_height)
                    # esc
                    elif c == '\x1b':
                        self._cf.commander.send_usb_disable()
                        for x in range(10):
                            self._cf.commander.send_hover_setpoint(0, 0, 0, target_height - (target_height - start_height) / 10.0 * x)
                            time.sleep(0.2)
                        break
                    else:
                        self._cf.commander.send_keep_alive()
                    print('vx: {}, vy: {}, height: {}'.format(vx, vy, target_height))

                except IOError: pass
                time.sleep(0.1)

        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

        cnt = 0
        while cnt < 15:
            self._cf.commander.send_setpoint(0, 0, 0, thrust_max - 2000)
            cnt = cnt + 1
            time.sleep(0.1)
            
        # cnt = 0
        # while cnt < 20:
        #     self._cf.commander.send_hover_setpoint(0, 0, 0, target_height)
        #     # self._cf.commander.send_hover_setpoint(0, 0, 0, start_height + (target_height - start_height) * (cnt / 100.0))
        #     # self._cf.commander.send_setpoint(0, 0, 0, thrust)
        #     # self._cf.commander.send_setpoint(0, 0, 0, 18000)
        #     cnt = cnt + 1
        #     time.sleep(0.1)

        # print("move")
        # cnt = 0
        # while cnt < 20:
        #     self._cf.commander.send_hover_setpoint(0.3, 0, 0, target_height) # (forward, left)
        #     cnt = cnt + 1
        #     time.sleep(0.1)

        # cnt = 0
        # while cnt < 40:
        #     self._cf.commander.send_hover_setpoint(0, 0, 0, target_height) # (forward, left)
        #     cnt = cnt + 1
        #     time.sleep(0.1)

        # print("down")
        # cnt = 0
        # while cnt < 10:
        #     self._cf.commander.send_hover_setpoint(0, 0, 0, start_height)
        #     cnt += 1
        #     time.sleep(0.1)


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
        time.sleep(2)
        # self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    le = MotorRampExample(uri)
