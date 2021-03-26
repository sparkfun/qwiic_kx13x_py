#-----------------------------------------------------------------------------
# qwiic_kx13x.py
#
# Python library for the SparkFun qwiic KX13X sensor.
#
# This sensor is available on the SparkFun Environmental Combo Breakout board.
#   https://www.sparkfun.com/products/14348
#
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, May 2019
#
# This python library supports the SparkFun Electroncis qwiic
# qwiic sensor/board ecosystem
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#==================================================================================
#
# This is mostly a port of existing Arduino functionaly, so pylint is sad.
# The goal is to keep the public interface pthonic, but internal is internal
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

"""
qwiic_kx13x
============
Python module for the qwiic bme280 sensor, which is part of the [SparkFun Qwiic Environmental Combo Breakout](https://www.sparkfun.com/products/14348)
This python package is a port of the existing [SparkFun KX13X Arduino Library](https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library)
This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)
New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).
"""

#-----------------------------------------------------------------------------
from __future__ import print_function
import math
import qwiic_i2c

# Define the device name and I2C addresses. These are set in the class defintion
# as class variables, making them avilable without having to create a class instance.
# This allows higher level logic to rapidly create a index of qwiic devices at
# runtine
#
# The name of this device
_DEFAULT_NAME = "Qwiic KX13X"

# Some devices have multiple availabel addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the
# device.
_AVAILABLE_I2C_ADDRESS = [0x1F, 0x1E]

# Default Setting Values

# Part ID identifying KX132 and KX134 respectively
_WHO_AM_I = [0x3D, 0x46]

# define the class that encapsulates the device being created. All information associated with this
# device is encapsulated by this class. The device class should be the only value exported
# from this module.

class QwiicKX13XCore(object):
    """
    QwiicKX13XCore
        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The KX13X device object.
        :rtype: Object
    """
    # Constructor
    device_name         =_DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    TOTAL_ACCEL_DATA_16BIT = 6
    TOTAL_ACCEL_DATA_8BIT  = 3
    MAX_BUFFER_LENGTH      = 32

    XLSB = 0
    XMSB = 1
    YLSB = 2
    YMSB = 3
    ZLSB = 4
    ZMSB = 5

    DEFAULT_SETTINGS     =  0xC0
    INT_SETTINGS         =  0xE0
    SOFT_INT_SETTINGS    =  0xE1
    BUFFER_SETTINGS      =  0xE2
    TILT_SETTINGS        =  0xE3

    COTR_DEF_STATE       =  0x55
    COTR_POS_STATE       =  0xAA

    BUFFER_16BIT_SAMPLES =  0x01
    BUFFER_8BIT_SAMPLES  =  0x00
    BUFFER_MODE_FIFO     =  0x00
    BUFFER_MODE_STREAM   =  0x01
    BUFFER_MODE_TRIGGER  =  0x02


    # Register names for the KX13X

    KX13X_MAN_ID           = 0x00
    KX13X_PART_ID          = 0x01
    KX13X_XADP_L           = 0x02
    KX13X_XADP_H           = 0x03
    KX13X_YADP_L           = 0x04
    KX13X_YADP_H           = 0x05
    KX13X_ZADP_L           = 0x06
    KX13X_ZADP_H           = 0x07
    KX13X_XOUT_L           = 0x08
    KX13X_XOUT_H           = 0x09
    KX13X_YOUT_L           = 0x0A
    KX13X_YOUT_H           = 0x0B
    KX13X_ZOUT_L           = 0x0C
    KX13X_ZOUT_H           = 0x0D
    # 0x0E - 0x11 Reserved
    KX13X_COTR             = 0x12
    KX13X_WHO_AM_I         = 0x13
    KXI3X_TSCP             = 0x14
    KX13X_TSPP             = 0x15
    KX13X_INS1             = 0x16
    KX13X_INS2             = 0x17
    KX13X_INS3             = 0x18
    KX13X_STATUS_REG       = 0x19
    KX13X_INT_REL          = 0x1A
    KX13X_CNTL1            = 0x1B
    KX13X_CNTL2            = 0x1C
    KX13X_CNTL3            = 0x1D
    KX13X_CNTL4            = 0x1E
    KX13X_CNTL5            = 0x1F
    KX13X_CNTL6            = 0x20
    KX13X_ODCNTL           = 0x21
    KX13X_INC1             = 0x22
    KX13X_INC2             = 0x23
    KX13X_INC3             = 0x24
    KX13X_INC4             = 0x25
    KX13X_INC5             = 0x26
    KX13X_INC6             = 0x27
    # 0x28 Reserved
    KX13X_TILT_TIMER       = 0x29
    KX13X_TDTRC            = 0x2A
    KX13X_TDTC             = 0x2B
    KX13X_TTH              = 0x2C
    KX13X_TTL              = 0x2D
    KX13X_FTD              = 0x2E
    KX13X_STD              = 0x2F
    KX13X_TLT              = 0x30
    KX13X_TWS              = 0x31
    KX13X_FFTH             = 0x32
    KX13X_FFC              = 0x33
    KX13X_FFCNTL           = 0x34
    # 0x35 - 0x36 Reserved
    KX13X_TILT_ANGLE_LL    = 0x37
    KX13X_TILT_ANGLE_HL    = 0x38
    KX13X_HYST_SET         = 0x39
    KX13X_LP_CNTL1         = 0x3A
    KX13X_LP_CNTL2         = 0x3B
    # 0x3C - 0x48 Reserved
    KX13X_WUFTH            = 0x49
    KX13X_BTSWUFTH         = 0x4A
    KX13X_BTSTH            = 0x4B
    KX13X_BTSC             = 0x4C
    KX13X_WUFC             = 0x4D
    # 0x4E - 0x5C Reserved
    KX13X_SELF_TEST        = 0x5D
    KX13X_BUF_CNTL1        = 0x5E
    KX13X_BUF_CNTL2        = 0x5F
    KX14X_BUF_STATUS_1     = 0x60
    KX13X_BUF_STATUS_2     = 0x61
    KX13X_BUF_CLEAR        = 0x62
    KX13X_BUF_READ         = 0x63
    KX13X_ADP_CNTL1        = 0x64
    KX13X_ADP_CNTL2        = 0x65
    KX13X_ADP_CNTL3        = 0x66
    KX13X_ADP_CNTL4        = 0x67
    KX13X_ADP_CNTL5        = 0x68
    KX13X_ADP_CNTL6        = 0x69
    KX13X_ADP_CNTL7        = 0x6A
    KX13X_ADP_CNTL8        = 0x6B
    KX13X_ADP_CNTL9        = 0x6C
    KX13X_ADP_CNTL10       = 0x6D
    KX13X_ADP_CNTL11       = 0x6E
    KX13X_ADP_CNTL12       = 0x6F
    KX13X_ADP_CNTL13       = 0x70
    KX13X_ADP_CNTL14       = 0x71
    KX13X_ADP_CNTL15       = 0x72
    KX13X_ADP_CNTL16       = 0x73
    KX13X_ADP_CNTL17       = 0x74
    KX13X_ADP_CNTL18       = 0x75
    KX13X_ADP_CNTL19       = 0x76
    # Reserved 0x77 - 0x7F



    KX13X_SUCCESS       = 0x00
    KX13X_GENERAL_ERROR = 0x01
    KX13X_I2C_ERROR     = 0x02


    # HARDWARE_INTERRUPTS

    HI_TILT_POSITION  = 0x01
    HI_WAKE_UP        = 0x02
    HI_TAP_DOUBLE_TAP = 0x04
    HI_BACK_TO_SLEEP  = 0x08
    HI_DATA_READY     = 0x10
    HI_WATERMARK      = 0x20
    HI_BUFFER_FULL    = 0x40
    HI_FREEFALL       = 0x80

    output_data = {xData:0, yData:0, zData:0}

    raw_output_data = {xData:0, ydata:0, zData:0}

    # Constructor
    def __init__(self, address=None, i2c_driver=None):

        # Did the user specify an I2C address?
        self.address = self.available_addresses[0] if address is None else address

        # load the I2C driver if one isn't provided

        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

    # ----------------------------------
    # is_connected()
    #
    # Is an actual board connected to our system?

    def is_connected(self):
        """
            Determine if a KX13X device is conntected to the system..
            :return: True if the device is connected, otherwise False.
            :rtype: bool
        """
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(is_connected)

    # ----------------------------------
    # begin()
    #
    # Initialize the system/validate the board.
    def beginCore(self):
        """
            Initialize the operation of the KX13X module
            :return: Returns true of the initializtion was successful, otherwise False.
            :rtype: bool
        """
        # are we who we need to be?
        chipID = self._i2c.readByte(self.address, self.KX13X_WHO_AM_I)
        if chipID not in _WHO_AM_I:
            print("Invalid Chip ID: 0x" % chipID)
            return False

        return True

    def initialize(self, settings = DEFAULT_SETTINGS):
        """
            Does something
            :param:
                :return:

        """

    def run_command_test(self):
        """
            Does something
            :param:
                :return:

        """
    def accel_control(self, enable):
        """
            Does something
            :param:
                :return:

        """
        # Make the mode a property of this object
        #mode = property(get_mode, set_mode)
    
    def get_accel_state(self):
        """
            Does something
            :param:
                :return:

        """
    #temperature_celsius = property(get_temperature_celsius)

    def set_range(self):
        """
            Does something
            :param:
                :return:

        """

    def set_output_data_rate(self, odr):
        """
            Does something
            :param:
                :return:

        """
    
    def read_output_data_rate(self)
        """
            Does something
            :param:
                :return:

        """
    
    def set_interrupt_pin(self, enable, polarity = 0, pulse_width = 0, 
                          latch_control = False):
        """
            Does something
            :param:
                :return:

        """
    def route_hardware_interrupt(self, rdr, pin):
        """
            Does something
            :param:
                :return:

        """
    def clear_interrupt(self):
        """
            Does something
            :param:
                :return:

        """
    def data_trigger(self):
        """
            Does something
            :param:
                :return:

        """
    def set_buffer_threshold(self, threshold):
        """
            Does something
            :param:
                :return:

        """
    def set_buffer_operation(self, operation_mode, resolution):
        """
            Does something
            :param:
                :return:

        """
    def enable_buffer(self, enable, enable_interrupt):
        """
            Does something
            :param:
            :return:

        """
    def get_raw_accel_data(self, raw_output_data):
        """
            Does something
            :param:
            :return:

        """

class QwiicKX132(object):
    def __init__(self):
    def begin(self):
        """
            Does something
            :param:
            :return:

            """
    def get_accel_data(self):
        """
            Does something
            :param:
            :return:

            """
    def conv_accel_data(self):
        """
            Does something
            :param:
            :return:

            """

class QwiicKX134(object):

    def __init__(self):

    def begin(self):
        """
            Does something
            :param:
            :return:

            """
    def get_accel_data(self):
        """
            Does something
            :param:
            :return:

            """
    def conv_accel_data(self):
        """
            Does something
            :param:
            :return:

            """

