#!/usr/bin/env python

import sys
import time
import binascii
import os
import threading
import struct

import bluepy
import yaml
from .util import *

#Service UUIDs
RobotControlService = "22bb746f2ba075542d6f726568705327"
BLEService = "22bb746f2bb075542d6f726568705327"
AntiDosCharacteristic = "22bb746f2bbd75542d6f726568705327"
TXPowerCharacteristic = "22bb746f2bb275542d6f726568705327"
WakeCharacteristic = "22bb746f2bbf75542d6f726568705327"
ResponseCharacteristic = "22bb746f2ba675542d6f726568705327"
CommandsCharacteristic = "22bb746f2ba175542d6f726568705327"

#These are the message response code that can be return by Sphero.
MRSP = dict(
  ORBOTIX_RSP_CODE_OK = 0x00,           #Command succeeded
  ORBOTIX_RSP_CODE_EGEN = 0x01,         #General, non-specific error
  ORBOTIX_RSP_CODE_ECHKSUM = 0x02,      #Received checksum failure
  ORBOTIX_RSP_CODE_EFRAG = 0x03,        #Received command fragment
  ORBOTIX_RSP_CODE_EBAD_CMD = 0x04,     #Unknown command ID
  ORBOTIX_RSP_CODE_EUNSUPP = 0x05,      #Command currently unsupported
  ORBOTIX_RSP_CODE_EBAD_MSG = 0x06,     #Bad message format
  ORBOTIX_RSP_CODE_EPARAM = 0x07,       #Parameter value(s) invalid
  ORBOTIX_RSP_CODE_EEXEC = 0x08,        #Failed to execute command
  ORBOTIX_RSP_CODE_EBAD_DID = 0x09,     #Unknown device ID
  ORBOTIX_RSP_CODE_POWER_NOGOOD = 0x31, #Voltage too low for refash operation
  ORBOTIX_RSP_CODE_PAGE_ILLEGAL = 0x32, #Illegal page number provided
  ORBOTIX_RSP_CODE_FLASH_FAIL = 0x33,   #Page did not reprogram correctly
  ORBOTIX_RSP_CODE_MA_CORRUPT = 0x34,   #Main application corrupt
  ORBOTIX_RSP_CODE_MSG_TIMEOUT = 0x35)  #Msg state machine timed out


#ID codes for asynchronous packets
IDCODE = dict(
  PWR_NOTIFY = chr(0x01),               #Power notifications
  LEVEL1_DIAG = chr(0x02),              #Level 1 Diagnostic response
  DATA_STRM = chr(0x03),                #Sensor data streaming
  CONFIG_BLOCK = chr(0x04),             #Config block contents
  SLEEP = chr(0x05),                    #Pre-sleep warning (10 sec)
  MACRO_MARKERS =chr(0x06),             #Macro markers
  COLLISION = chr(0x07))                #Collision detected

RECV = dict(
  ASYNC = [chr(0xff), chr(0xfe)],
  SYNC = [chr(0xff), chr(0xff)])

#did and cid
REQ = dict(
  WITH_RESPONSE =[0xff, 0xff],
  WITHOUT_RESPONSE =[0xff, 0xfe],
  CMD_PING = [0x00, 0x01],
  CMD_VERSION = [0x00, 0x02],
  CMD_SET_BT_NAME = [0x00, 0x10],
  CMD_GET_BT_NAME = [0x00, 0x11],
  CMD_SET_AUTO_RECONNECT = [0x00, 0x12],
  CMD_GET_AUTO_RECONNECT = [0x00, 0x13],
  CMD_GET_PWR_STATE = [0x00, 0x20],
  CMD_SET_PWR_NOTIFY = [0x00, 0x21],
  CMD_SLEEP = [0x00, 0x22],
  CMD_GOTO_BL = [0x00, 0x30],
  CMD_RUN_L1_DIAGS = [0x00, 0x40],
  CMD_RUN_L2_DIAGS = [0x00, 0x41],
  CMD_CLEAR_COUNTERS = [0x00, 0x42],
  CMD_ASSIGN_COUNTER = [0x00, 0x50],
  CMD_POLL_TIMES = [0x00, 0x51],
  CMD_SET_HEADING = [0x02, 0x01],
  CMD_SET_STABILIZ = [0x02, 0x02],
  CMD_SET_ROTATION_RATE = [0x02, 0x03],
  CMD_SET_APP_CONFIG_BLK = [0x02, 0x04],
  CMD_GET_APP_CONFIG_BLK = [0x02, 0x05],
  CMD_SET_DATA_STRM = [0x02, 0x11],
  CMD_CFG_COL_DET = [0x02, 0x12],
  CMD_SET_RGB_LED = [0x02, 0x20],
  CMD_SET_BACK_LED = [0x02, 0x21],
  CMD_GET_RGB_LED = [0x02, 0x22],
  CMD_ROLL = [0x02, 0x30],
  CMD_BOOST = [0x02, 0x31],
  CMD_SET_RAW_MOTORS = [0x02, 0x33],
  CMD_SET_MOTION_TO = [0x02, 0x34],
  CMD_GET_CONFIG_BLK = [0x02, 0x40],
  CMD_SET_DEVICE_MODE = [0x02, 0x42],
  CMD_SET_CFG_BLOCK = [0x02, 0x43],
  CMD_GET_DEVICE_MODE = [0x02, 0x44],
  CMD_RUN_MACRO = [0x02, 0x50],
  CMD_SAVE_TEMP_MACRO = [0x02, 0x51],
  CMD_SAVE_MACRO = [0x02, 0x52],
  CMD_DEL_MACRO = [0x02, 0x53],
  CMD_INIT_MACRO_EXECUTIVE = [0x02, 0x54],
  CMD_ABORT_MACRO = [0x02, 0x55],
  CMD_GET_MACRO_STATUS = [0x02, 0x56],
  CMD_SET_MACRO_STATUS = [0x02, 0x57])

STRM_MASK1 = dict(
  GYRO_H_FILTERED    = 0x00000001,
  GYRO_M_FILTERED    = 0x00000002,
  GYRO_L_FILTERED    = 0x00000004,
  LEFT_EMF_FILTERED  = 0x00000020,
  RIGHT_EMF_FILTERED = 0x00000040,
  MAG_Z_FILTERED     = 0x00000080,
  MAG_Y_FILTERED     = 0x00000100,
  MAG_X_FILTERED     = 0x00000200,
  GYRO_Z_FILTERED    = 0x00000400,
  GYRO_Y_FILTERED    = 0x00000800,
  GYRO_X_FILTERED    = 0x00001000,
  ACCEL_Z_FILTERED   = 0x00002000,
  ACCEL_Y_FILTERED   = 0x00004000,
  ACCEL_X_FILTERED   = 0x00008000,
  IMU_YAW_FILTERED   = 0x00010000,
  IMU_ROLL_FILTERED  = 0x00020000,
  IMU_PITCH_FILTERED = 0x00040000,
  LEFT_EMF_RAW       = 0x00200000,
  RIGHT_EMF_RAW      = 0x00400000,
  MAG_Z_RAW          = 0x00800000,
  MAG_Y_RAW          = 0x01000000,
  MAG_X_RAW          = 0x02000000,
  GYRO_Z_RAW         = 0x04000000,
  GYRO_Y_RAW         = 0x08000000,
  GYRO_X_RAW         = 0x10000000,
  ACCEL_Z_RAW        = 0x20000000,
  ACCEL_Y_RAW        = 0x40000000,
  ACCEL_X_RAW        = 0x80000000)

STRM_MASK2 = dict(
  QUATERNION_Q0      = 0x80000000,
  QUATERNION_Q1      = 0x40000000,
  QUATERNION_Q2      = 0x20000000,
  QUATERNION_Q3      = 0x10000000,
  ODOM_X             = 0x08000000,
  ODOM_Y             = 0x04000000,
  ACCELONE           = 0x02000000,
  VELOCITY_X         = 0x01000000,
  VELOCITY_Y         = 0x00800000)

class DelegateObj(bluepy.btle.DefaultDelegate):
    """
    Delegate object that get calls when there is a notification
    """
    def __init__(self, sphero_obj,lock):
        bluepy.btle.DefaultDelegate.__init__(self)
        self._sphero_obj = sphero_obj
        self._callback_dict = {}
        self._wait_list = {}
        self._data_group_callback = {}
        self._enabled_group = []
        self._buffer_bytes = b''
        self._notification_lock = lock


class Sphero(threading.Thread):   #object

    RAW_MOTOR_MODE_OFF = "00"
    RAW_MOTOR_MODE_FORWARD = "01"
    RAW_MOTOR_MODE_REVERSE = "02"
    RAW_MOTOR_MODE_BRAKE = "03"
    RAW_MOTOR_MODE_IGNORE = "04"


    # def con_b(n, length, endianess='big'):
    #     h = '%x' % n
    #     s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    #     return s if endianess == 'big' else s[::-1]

    def __init__(self, addr=None):
        threading.Thread.__init__(self)

        if(addr == None):
            #search for sphero
            sphero_list = search_for_sphero()
            if(len(sphero_list) == 0):
                raise "No Sphero Found in Vicinity"
            addr = sphero_list[0]

        self._addr = addr
        self._connected = False
        self.seq=0
        self._stream_rate = 10
        self.shutdown = False
        #load the mask list

        with open(os.path.join(os.path.dirname(__file__),'data','mask_list.yaml'),'r') as mask_file:
             self._mask_list = yaml.load(mask_file)
        self._curr_data_mask = bytearray.fromhex("0000 0000")

        self._notification_lock = threading.RLock() #RLock
        #start a listener loop

    def connect(self):
        """
        Connects the sphero with the address given in the constructor
        """
        self._device = bluepy.btle.Peripheral(self._addr, addrType=bluepy.btle.ADDR_TYPE_RANDOM)
        self._notifier = DelegateObj(self, self._notification_lock)
        #set notifier to be notified
        self._device.withDelegate(self._notifier)

        self._devModeOn()
        self._connected = True #Might need to change to be a callback format
        #get the command service
        cmd_service = self._device.getServiceByUUID(RobotControlService)
        self._cmd_characteristics = {}
        characteristic_list = cmd_service.getCharacteristics()
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            self._cmd_characteristics[uuid_str] = characteristic

        self._listening_flag = True
        self._listening_thread = threading.Thread(target=self._listening_loop)
        self._listening_thread.start()
        return True

    def _devModeOn(self):
        """
        A sequence of read/write that enables the developer mode
        """
        service = self._device.getServiceByUUID(BLEService)
        characteristic_list = service.getCharacteristics()
        #make it into a dict
        characteristic_dict = {}
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            characteristic_dict[uuid_str] = characteristic

        characteristic = characteristic_dict[AntiDosCharacteristic]
        characteristic.write("011i3".encode()) #removed True
        characteristic = characteristic_dict[TXPowerCharacteristic]
        characteristic.write('\x0007') #removed True
        characteristic = characteristic_dict[WakeCharacteristic]
        characteristic.write('\x01')

    def send(self,cmd,data,resp):
      # Packets are sent from Client -> Sphero in the following byte format::
      # -------------------------------------------------------
      # | SOP1 | SOP2 | DID | CID | SEQ | DLEN | <data> | CHK |
      # -------------------------------------------------------

        packed_data=self.pack_cmd(cmd,data)
        checksum =~ sum(packed_data) % 256
        if resp:
          output = REQ['WITH_RESPONSE'] + packed_data + [checksum]
        else:
          output = REQ['WITHOUT_RESPONSE'] + packed_data + [checksum]

        with self._notification_lock:
          self._cmd_characteristics[CommandsCharacteristic].write(''.join(struct.pack('B',x) for x in output))
        return self.seq

    def _listening_loop(self):
        pass
        #while(self._listening_flag):
            #with self._notification_lock:
                #self._device.waitForNotifications(0.001)

    """functions for packing data"""

    def pack_cmd(self, req ,cmd):
      self.inc_seq()
      return req + [self.seq] + [len(cmd)+1] + cmd

    def inc_seq(self):
      self.seq = self.seq + 1
      if self.seq > 0xff:
        self.seq = 0

    def clamp(self, n, minn, maxn):
      """Function for ensuring data is in range"""
      return max(min(maxn, n), minn)

    def _format_data_array(self, arr):
        """
        helper function that converts int or string to bytes
        """
        if isinstance(arr,list):
            for i,value in enumerate(arr):
                if isinstance(value,int):
                    arr[i] = to_bytes(value,1,'big')
                    print(repr("%s"%arr[i]))
        return arr

    """ CORE functionality """

    def ping(self,resp=True):
        return self.send(REQ['CMD_PING'],[],resp)

    """ Sphero functionality """

    def roll(self, speed, heading,state, resp=False):
        """
        This commands Sphero to roll along the provided vector. Both a
        speed and a heading are required; the latter is considered
        relative to the last calibrated direction. A state Boolean is also
        provided (on or off). The client convention for heading follows the 360
        degrees on a circle, relative to the ball: 0 is straight ahead, 90
        is to the right, 180 is back and 270 is to the left. The valid
        range is 0..359.
        :param speed: 0-255 value representing 0-max speed of the sphero.
        :param heading: heading in degrees from 0 to 359.
        :param state: 00h for off (braking) and 01h for on (driving).
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_ROLL'],[self.clamp(speed,0,255), (heading>>8), (heading & 0xff), state], resp=resp)

    def set_heading(self, heading, resp=False):
        """
        This allows the client to adjust the orientation of Sphero by
        commanding a new reference heading in degrees, which ranges from 0
        to 359. You will see the ball respond immediately to this command
        if stabilization is enabled.
        :param heading: heading in degrees from 0 to 359 (motion will be\
        shortest angular distance to heading command)
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_HEADING'],[(heading>>8),(heading & 0xff)], resp=resp)


    def set_rgb_led(self, red, green, blue, save, resp=False):
        """
        This allows you to set the RGB LED color. The composite value is
        stored as the "application LED color" and immediately driven to
        the LED (if not overridden by a macro or orbBasic operation). If
        FLAG is true, the value is also saved as the "user LED color"
        which persists across power cycles and is rendered in the gap
        between an application connecting and sending this command.
        :param red: red color value.
        :param green: green color value.
        :param blue: blue color value.
        :param save: 01h for save (color is saved as "user LED color").
        """
        self.send(REQ['CMD_SET_RGB_LED'], [self.clamp(red,0,255), self.clamp(green,0,255), self.clamp(blue,0,255), save], resp=resp)

    def set_stabilization(self,bool_flag, resp=False):
        """
        This turns on or off the internal stabilization of Sphero, in
        which the IMU is used to match the ball's orientation to its
        various set points. The flag value is as you would expect, 00h for
        off and 01h for on.
        :param enable: 00h for off and 01h for on (on by default).
        :param response: request response back from Sphero.
        """
        data = [0x01 if bool_flag else 0x00]
        self.send(REQ['CMD_SET_STABILIZ'],data, resp=resp)


    def set_raw_motor_values(self,lmode,lpower,rmode,rpower, resp=False):
        """
        This allows you to take over one or both of the motor output
        values, instead of having the stabilization system control
        them. Each motor (left and right) requires a mode (see below) and
        a power value from 0- 255. This command will disable stabilization
        if both modes aren't "ignore" so you'll need to re-enable it via
        CID 02h once you're done.
        :param mode: 0x00 - off, 0x01 - forward, 0x02 - reverse, 0x03 -\
        brake, 0x04 - ignored.
        :param power: 0-255 scalar value (units?).
        """
        data = [lmode, int(lpower), rmode, int(rpower)]

        #By default, we going to cancel it
        self.send(REQ['CMD_SET_RAW_MOTORS'],data, resp=resp)

    def set_rotation_rate(self, rate, resp):
        """
        This allows you to control the rotation rate that Sphero will use
        to meet new heading commands. The commanded value is in units of
        0.784 degrees/sec. So, setting a value of c8h will set the
        rotation rate to 157 degrees/sec. A value of 255 jumps to the
        maximum and a value of 1 is the minimum.
        :param rate: rotation rate in units of 0.784degrees/sec (setting\
        this value will not cause the device to move only set the rate it\
        will move in other funcation calls).
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_ROTATION_RATE'],[self.clamp(rate, 0, 255)], resp)

    def disconnect(self):
        self.is_connected = False
        return self.is_connected



