#!/usr/bin/env python

import sys
import time
import binascii
import os
import operator
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
  CMD_LOCATOR= [0x02, 0x13],  #dodano configure
  CMD_READ_LOCATOR= [0x02, 0x15],

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

LOC_MASK = dict(
  VELOCITY_X         = 0x01000000,
  VELOCITY_Y         = 0x00800000,
  ODOM_X             = 0x08000000,
  ODOM_Y             = 0x04000000
  )

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

        #init callback dictionaries
        self._async_callback_dict = dict()
        self._sync_callback_dict = dict()

    def register_callback(self, seq, callback):
        self._callback_dict[seq] = callback

    def register_async_callback(self, group_name, callback):
        self._data_group_callback[group_name] = callback
        self._enabled_group = list(set(self._enabled_group) | set([group_name]))

    def handle_callbacks(self, packet):
        #unregister callback
        callback = self._callback_dict.pop(packet[3])
        MRSP = packet[2]
        dlen = (packet[4] - 1)
        data = []
        if(dlen > 0):
            data = packet[5:5+dlen]
        #parse the packet
        callback(MRSP, data)

    def wait_for_resp(self,seq,timeout=None):
        #function waits for a response in the handle notification part
        self._wait_list[seq] = None;
        while(self._wait_list[seq] == None):
            with self._notification_lock:
                self._sphero_obj._device.waitForNotifications(0.05)
        return self._wait_list.pop(seq)

    def wait_for_sim_response(self, seq, timeout=None):
        #function waits for a response in the handle notification part
        self._wait_list[seq] = None;
        while(self._wait_list[seq] == None):
            with self._notification_lock:
                self._sphero_obj._device.waitForNotifications(0.05)
        data = self._wait_list.pop(seq)
        return (len(data) == 6 and data[0] == 255)    

    def parse_single_pack(self, data):
        if self._sphero_obj.is_connected:
          if(data[1] == '\xff'): #255

              #get the sequence number and check if a callback is assigned
              if(data[3] in self._callback_dict):
                  self.handle_callbacks(data)
              #check if we have it in the wait list
              elif(from_bytes(data[3],'big') in self._wait_list):
                  self._wait_list[from_bytes(data[3],'big')] = data
              #simple response
              elif(len(data) == 6 and data[0] == 255 and data[2] == 0):
                  pass
                  #print("receive simple response for seq:{}".format(data[3]))
              else:
                  print("unknown response:{}".format(data))
              #Sync Message
          elif(data[1] == '\xfe'): #254
              #Async Message
              data_length = (ord(data[3])<<8)+ord(data[4])
              data_packet = data[:(5+data_length)]

              if(data[2] == '\x03'):
                self._data_group_callback['\x03'](self.parse_data_strm(data_packet, data_length))
              elif (data[2]=='\x01'): #and self._enabled_group.has_key(IDCODE['PWR_NOTIFY'])):
                self._data_group_callback['\x01'](self.parse_pwr_notify(data_packet, data_length))
              else:
                print("unknown async response:{}".format(data))
          else:
              pass

    def handleNotification(self, cHandle, data):
        #merge the data with previous incomplete instance
        self._buffer_bytes =  self._buffer_bytes + data

        #loop through it and see if it's valid
        while(len(self._buffer_bytes) > 5): #we need at least 6 bytes
                #split the data until it's a valid chunk
                i=0
                len_int=0
                chks_pckt=0
                if ((self._buffer_bytes[0]=='\xff') and ((self._buffer_bytes[1]=='\xff') or (self._buffer_bytes[1]=='\xfe'))):
                    #assuming we found the start od the packet
                    if (self._buffer_bytes[i+1]=='\xff'):
                      #sync
                      len_int=ord(self._buffer_bytes[i+4])
                      chks_pckt=cal_packet_checksum(self._buffer_bytes[i+2:i+5+len_int-1]) #not sure 5 or 6 
                      if (len(self._buffer_bytes)>=(len_int+5)):
                        if (chks_pckt==ord(self._buffer_bytes[4+len_int])):
                          #data valid
                          data_s_pack=self._buffer_bytes[0:5+len_int]
                          self._buffer_bytes=self._buffer_bytes[len_int+5:]
                          self.parse_single_pack(data_s_pack)
                        else:
                          #checksum not good
                          self._buffer_bytes=self._buffer_bytes[1:]
                      else:
                        break

                    elif (self._buffer_bytes[i+1]=='\xfe'):
                      #async
                      len_int=(ord(self._buffer_bytes[i+3])<<8)+ord(self._buffer_bytes[i+4])
                      chks_pckt=cal_packet_checksum(self._buffer_bytes[i+2:i+5+len_int-1]) #-1 for excluding checksum 
                      if (len(self._buffer_bytes)>=(len_int+5)):
                        if (chks_pckt==ord(self._buffer_bytes[4+len_int])):
                          #data valid
                          data_s_pack=self._buffer_bytes[0:4+len_int]
                          self._buffer_bytes=self._buffer_bytes[len_int+4:]
                          self.parse_single_pack(data_s_pack)
                        else:
                          #checksum not good
                          self._buffer_bytes=self._buffer_bytes[1:]
                      elif(len_int>300):
                        self._buffer_bytes=self._buffer_bytes[1:]
                      else:
                        break
                else:
                    self._buffer_bytes=self._buffer_bytes[1:]


    def parse_pwr_notify(self, data, data_length):
      """
      The data payload of the async message is 1h bytes long and
      formatted as follows::
        --------
        |State |
        --------
      The power state byte: 
        * 01h = Battery Charging, 
        * 02h = Battery OK,
        * 03h = Battery Low, 
        * 04h = Battery Critical
      """
      return struct.unpack_from('B', ''.join(data[5:]))[0]

    def parse_data_strm(self, data, data_length):
      output={}
      for i in range((data_length-1)/2):
        unpack = struct.unpack_from('>h', ''.join(data[5+2*i:]))
        output[self._sphero_obj.mask_list[i]] = unpack[0]
      return output

class Sphero(threading.Thread):   #object

    RAW_MOTOR_MODE_OFF = "00"
    RAW_MOTOR_MODE_FORWARD = "01"
    RAW_MOTOR_MODE_REVERSE = "02"
    RAW_MOTOR_MODE_BRAKE = "03"
    RAW_MOTOR_MODE_IGNORE = "04"

    def __init__(self, addr=None):
        threading.Thread.__init__(self)

        if(addr == None):
            #search for sphero
            sphero_list = search_for_sphero()
            if(len(sphero_list) == 0):
                raise "No Sphero Found in Vicinity"
            addr = sphero_list[0]

        self._addr = addr
        self.is_connected = False
        self.seq=0
        self._stream_rate = 10
        self.shutdown = False
        #load the mask list
        self.stream_mask1 = None
        self.stream_mask2 = None

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
        self.is_connected = True #Might need to change to be a callback format
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
        return arr

    def create_mask_list(self, mask1, mask2):
        #save the mask
        sorted_STRM1 = sorted(STRM_MASK1.iteritems(), key=operator.itemgetter(1), reverse=True)
        #create a list containing the keys that are part of the mask
        self.mask_list1 = [key  for key, value in sorted_STRM1 if value & mask1]

        sorted_STRM2 = sorted(STRM_MASK2.iteritems(), key=operator.itemgetter(1), reverse=True)
        #create a list containing the keys that are part of the mask
        self.mask_list2 = [key  for key, value in sorted_STRM2 if value & mask2]
        self.mask_list = self.mask_list1 + self.mask_list2

    """ CORE functionality """

    def ping(self,resp=True):
        return self.send(REQ['CMD_PING'],[],resp)

    def get_device_name(self,resp):
        seq_num = self.send(REQ['CMD_GET_BT_NAME'],[],resp)
        response = self._notifier.wait_for_resp(seq_num)
        name_data = {}
        name_data["name"] = response[5:12].decode('utf-8') #21
        name_data["bta"]=response[21:33].decode('utf-8')
        name_data["color"]=response[33:36].decode("utf-8")
        return name_data

    def set_locator(self,resp):
        self.send(REQ['CMD_LOCATOR'],[1,0,0,0],resp) #'\x00\x00\x00'


    def read_locator(self,resp):
        seq_num=self.send(REQ['CMD_READ_LOCATOR'],[],resp)
        response = self._notifier.wait_for_resp(seq_num)
        mask_locator = [key  for key in LOC_MASK if (1)]
        loc={}
        output={}
        for i in range(4):
          unpack = struct.unpack_from('>h', ''.join(response[5+2*i:]))
          output[mask_locator[i]] = unpack[0]
        return output


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
        data = ["01" if bool_flag else "00"]
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

  #get commands

    def set_power_notify(self, enable, response):
        """
        This enables Sphero to asynchronously notify the Client
        periodically with the power state or immediately when the power
        manager detects a state change. Timed notifications arrive every 10
        seconds until they're explicitly disabled or Sphero is unpaired. The
        flag is as you would expect, 00h to disable and 01h to enable.
        :param enable: 00h to disable and 01h to enable power notifications.
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_PWR_NOTIFY'],[enable], response)

    def get_power_state(self, response):
        """
        This returns the current power state and some additional
        parameters to the Client.
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_GET_PWR_STATE'],[], response)

    def set_filtered_data_strm(self, sample_div, sample_frames, pcnt, response):
        """
        Helper function to add all the filtered data to the data strm
        mask, so that the user doesn't have to set the data strm manually.

        :param sample_div: divisor of the maximum sensor sampling rate.
        :param sample_frames: number of sample frames emitted per packet.
        :param pcnt: packet count (set to 0 for unlimited streaming).
        :param response: request response back from Sphero.
        """
        mask1 = 0
        mask2 = 0
        for key,value in STRM_MASK1.iteritems():
          if 'FILTERED' in key:
            mask1 = mask1|value
        for value in STRM_MASK2.itervalues(): #bez key i itervalues
            mask2 = mask2|value
        self.set_data_strm(sample_div, sample_frames, mask1, pcnt, mask2, response)

    def set_data_strm(self, sample_div, sample_frames, sample_mask1, pcnt, sample_mask2, response):
        """
        Currently the control system runs at 400Hz and because it's pretty
        unlikely you will want to see data at that rate, N allows you to
        divide that down. sample_div = 2 yields data samples at 200Hz,
        sample_div = 10, 40Hz, etc. Every data sample consists of a
        "frame" made up of the individual sensor values as defined by the
        sample_mask. The sample_frames value defines how many frames to
        collect in memory before the packet is emitted. In this sense, it
        controls the latency of the data you receive. Increasing
        sample_div and the number of bits set in sample_mask drive the
        required throughput. You should experiment with different values
        of sample_div, sample_frames and sample_mask to see what works
        best for you.
        :param sample_div: divisor of the maximum sensor sampling rate.
        :param sample_frames: number of sample frames emitted per packet.
        :param sample_mask1: bitwise selector of data sources to stream.
        :param pcnt: packet count (set to 0 for unlimited streaming).
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_DATA_STRM'], \
          [(sample_div>>8), (sample_div & 0xff), (sample_frames>>8), (sample_frames & 0xff), ((sample_mask1>>24) & 0xff), \
          ((sample_mask1>>16) & 0xff),((sample_mask1>>8) & 0xff), (sample_mask1 & 0xff), pcnt, ((sample_mask2>>24) & 0xff), \
          ((sample_mask2>>16) & 0xff),((sample_mask2>>8) & 0xff), (sample_mask2 & 0xff)], response)

        self.create_mask_list(sample_mask1, sample_mask2)
        self.stream_mask1 = sample_mask1
        self.stream_mask2 = sample_mask2

  #callbacks

    def add_async_callback(self, callback_type, callback):
        self._notifier.register_async_callback(callback_type,callback)

    def disconnect(self):
        self.is_connected = False
        return self.is_connected



