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
Python module for the qwiic kx132/4 accelerometers.
This python package is a port of the existing [SparkFun KX13X Arduino Library](https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library)
This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)
New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).
"""

#-----------------------------------------------------------------------------
from __future__ import print_function
import qwiic_i2c
from collections import namedtuple

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
    device_name         = _DEFAULT_NAME
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

    raw_output_data = namedtuple('raw_output_data', 'x y z')

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

        return chipID

    def initialize(self, settings = DEFAULT_SETTINGS):
        """
            Initialize configures the accelerometer's registers into a number of
            different modes: asyncronous, hardware trigger, software trigger,
            and buffer.
            :param settings: A class constant indicating which setting to
                configure: DEFAULT_SETTINGS, INT_SETTINGS, SOFT_INT_SETTINGS,
                BUFFER_SETTINGS.
            :return: No return value.

        """
        self.accel_control(False)

        if settings == self.DEFAULT_SETTINGS:
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, self.DEFAULT_SETTINGS)
        elif settings == self.INT_SETTINGS:
            self.set_interrupt_pin(True, 1)
            self.route_hardware_interrupt(self.HI_DATA_READY)
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, self.INT_SETTINGS)
        elif settings == self.SOFT_INT_SETTINGS:
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, self.INT_SETTINGS)
        elif settings == self.BUFFER_SETTINGS:
            self.set_interrupt_pin(True, 1)
            self.route_hardware_interrupt(self.HI_BUFFER_FULL)
            self.set_buffer_operation(self.BUFFER_MODE_FIFO, self.BUFFER_16BIT_SAMPLES)
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, self.INT_SETTINGS)
        # Space fore more default settings

    def run_command_test(self):
        """
            This function runs the self test built into the accelerometer.
            :return: Returns true upon successful test, and false otherwise.
            :rtype: bool
        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL2)
        reg_val &= 0xBF
        reg_val |= (1 << 6)
        self._i2c.writeByte(self.address, self.KX13X_CNTL2 , reg_val)

        reg_val = self._i2c.readByte(self.address, self.KX13X_COTR)
        if reg_val == COTR_POS_STATE:
            return True
        else:
            return False


    def accel_control(self, enable):
        """
            This functions controls the accelerometers power on and off state.
            :param enable: True or false indicating power on or off
            respectively.
            :return: Returns false when an incorrect argumen has been passed.
            :rtype: bool
        """
        if enable != True and enable != False:
            return False

        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        reg_val &= 0x7F
        reg_val |= (enable << 7)
        self._i2c.writeByte(self.address, self.KX13X_CNTL1 , reg_val)

    def get_accel_state(self):
        """
            Retrieves the state of the accelerometer: on or off.
            :return: Returns bit indicating the accelerometers power state.
            :rtype: int
        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        return (reg_val & 0x80) >> 7
    #temperature_celsius = property(get_temperature_celsius)

    def set_range(self, kx13x_range):
        """
            Sets the range reported by the accelerometer. For the KX132, the
            range is from 2G - 16G and for the KX134 it's 8G - 32G.
            :param kx13x_range: Eight constants (four per version) represent values from zero to
            four indicating the range to be set:
                KX132_RANGE2G,
                KX132_RANGE4G,
                KX132_RANGE8G,
                KX132_RANGE16G
                KX134_RANGE8G,
                KX134_RANGE16G,
                KX134_RANGE32G,
                KX134_RANGE64G.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool

        """

        if kx13x_range < 0 or kx13x_range > 3:
            return False

        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        reg_val &= 0xE7
        reg_val |= (kx13x_range << 3)
        self._i2c.writeByte(self.address, self.KX13X_CNTL1 , reg_val)


    def set_output_data_rate(self, rate):
        """
            Sets the rate at which the accelerometer outputs data.
            :param rate: A value from zero to fifteen indicating which rate to
            set.
            :return: Returns false if an an incorrect argument is given.
            :rtype: bool

        """
        if rate < 0 or rate > 15:
            return False

        accel_state = self.get_accel_state()
        self.accel_control(False)

        reg_val = self._i2c.readByte(self.address, self.KX13X_ODCNTL)
        reg_val &= 0x40
        reg_val |= rate
        self._i2c.writeByte(self.address, self.KX13X_ODCNTL , reg_val)
        self.accel_control(accel_state)


    def get_output_data_rate(self):
        """
            Gets the accelerometers output data rate.
            :return: Accelerometer's data rate in hertz.
            :rtype: float

        """

        reg_val = self._i2c.readByte(self.address, self.KX13X_ODCNTL)
        reg_val &= 0x40
        return (0.78 * (2 * reg_val))

    output_data_rate = property(get_output_data_rate, set_output_data_rate)

    def set_interrupt_pin(self, enable, polarity = 0, pulse_width = 0,
                          latch_control = False):
        """
            Sets all of whether the data ready bit is reported to the hardware
            interrupt pin, the polarity of the signal (HIGH or LOW), the width
            of the pulse, and how the interrupt is cleared.
            :param enable: Sets hardware interrupt to "on" or "off".
            :param polarity: Sets the active state of the hardware pin - HIGH
            or LOW.
            :param pulse_width: Sets the width of the interrupt pulse.
            :param latch_control: Sets how the interrupt pin is cleared.
            :return: Returns false if an an incorrect argument is given.
            :rtype: bool

        """
        if enable != True and enable != False:
            return False
        if polarity != 1 and polarity != 0:
            return False
        if pulse_width != 1 and pulse_width != 0:
            return False
        if latch_control < 0 or latch_control > 4:
            return False

        accel_state = self.get_accel_state()
        self.accel_control(False)

        combined_arguments = (pulse_width << 6) | (enable << 5) | (polarity << 4) | (latch_control << 3)

        reg_val = self._i2c.readByte(self.address, self.KX13X_INC1)
        reg_val &= 0x07
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_INC1 , reg_val)



    def route_hardware_interrupt(self, rdr, pin = 1):
        """
            Determines which interrupt is reported: freefall, buffer full,
            watermark, data ready, back to sleep, tap/double tap, wakeup or
            tilt. Also which hardware pin its reported on: one or two.
            :param rdr: The interrupt to be reported.
            :param pin: The hardware pin on which the interrupt is reported.
            :return: Returns true after configuring the register and false if an an
            incorrect argument is given.
            :rtype: bool

        """
        if rdr < 0 or rdr > 128:
            return False
        if pin != 1 and pin != 2:
            return False

        accel_state = self.get_accel_state()
        self.accel_control(False)

        if pin == 1:
            self._i2c.writeByte(self.address, self.KX13X_INC4 , rdr)
            self.accel_control(accel_state)
            return True
        else:
            self._i2c.writeByte(self.address, self.KX13X_INC6 , rdr)
            self.accel_control(accel_state)
            return True

    def clear_interrupt(self):
        """
            Clears the interrupt.
            :return: No return value.

        """
        self._i2c.readByte(self.address, self.KX13X_INT_REL)

    def data_trigger(self):
        """
            Reads the register indicating whether data is ready to be read.
            :return: Returns true if data is ready to be read and false
            otherwise.
            :rtype: bool

        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_INS2)
        if reg_val & 0x10:
            return True
        else:
            return False

    def set_buffer_threshold(self, threshold):
        """
            Sets how many samples are stored in the buffer.
            :param threshold: The number of samples to be stored.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if threshold < 2 or threshold > 171:
            return False

        resolution = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        resolution &= 0x40
        resolution = resolution >> 6

        if threshold > 86 and resolution == 1: # At 16bit resolution - max samples: 86
            threshold == 86

        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL1, threshold)

    def set_buffer_operation(self, operation_mode, resolution):
        """
            Sets the mode and resolution of the samples stored in the buffer.
            :param operation_mode: Sets the mode:
                                   BUFFER_MODE_FIFO
                                   BUFFER_MODE_STREAM
                                   BUFFER_MODE_TRIGGER
            :param resolution: Sets the resolution of the samples, 8 or 16 bit.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if resolution < 0 or resolution > 1:
            return False
        if operation_mode < 0 or operation_mode > 2:
            return False

        combined_arguments = (resolution << 6) | operation_mode

        reg_val = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        reg_val &= 0xBC
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL2 , reg_val)

    def enable_buffer(self, enable, enable_interrupt):
        """
            Enables the buffer and whether the buffer triggers an interrupt
            when full.
            :param enable: Enables the buffer.
            :param enable: Enables the buffer's interrupt.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if enable != True and enable != False:
            return False
        if enable_interrupt != True and enable_interrupt != False:
            return False

        combined_arguments = (enable << 7) | (enable_interrupt << 5)

        reg_val = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        reg_val &= 0x5F
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL2 , reg_val)

    def get_raw_accel_data(self):
        """
            Checks which registers are storing acceleration data and retrieves
            it, storing it in a named tuple local to the class.
        """

        reg_val = self._i2c.readByte(self.address, self.KX13X_INC4)
        if reg_val & 0x40:
            accel_data = self._i2c.readBlock(self.address, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_16BIT)
            xData = (accel_data[self.XMSB] << 8) | accel_data[self.XLSB]
            yData = (accel_data[self.YMSB] << 8) | accel_data[self.YLSB]
            zData = (accel_data[self.ZMSB] << 8) | accel_data[self.ZLSB]
        else:
            accel_data = self._i2c.readBlock(self.address, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_8BIT)
            xData = accel_data[0]
            yData = accel_data[1]
            zData = accel_data[2]

        self.raw_output_data.x = xData
        self.raw_output_data.y = yData
        self.raw_output_data.z = zData


class QwiicKX132(QwiicKX13XCore):

    KX132_WHO_AM_I = 0x3D
    KX132_RANGE2G  = 0x00
    KX132_RANGE4G  = 0x01
    KX132_RANGE8G  = 0x02
    KX132_RANGE16G = 0x03
    CONV_2G =  .00006103518784142582
    CONV_4G =  .0001220703756828516
    CONV_8G =  .0002441407513657033
    CONV_16G = .0004882811975463118

    kx132_accel = namedtuple('kx132_accel', 'x y z')

    def __init__(self, address = None, i2c_driver = None):
        super().__init__(address, i2c_driver)

    def begin(self):
        """
            Checks that communication can be made with the QwiicKX132 by checking
            the WHO_AM_I register.
            :return: Returns true if WHO_AM_I value is the correct one and
            false otherwise.
            :rtype: bool
        """
        chipID = self.beginCore()
        if chipID == self.KX132_WHO_AM_I:
            return True
        else:
            return False

    def get_accel_data(self):
        """
            Retrieves acceleration data and converts it, storing it within a
            named tuple local to the QwiicKX132 class.
        """
        self.get_raw_accel_data()
        self.conv_accel_data()
def conv_accel_data(self):
        """
            Converts raw acceleration data according to the range setting and
            stores it in a named tuple local to the QwiicKX132.
        """
        accel_range = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        accel_range &= 0x18
        accel_range = accel_range >> 3

        if accel_range == self.KX132_RANGE2G:
            self.kx132_accel.x = round(self.raw_output_data.x * self.CONV_2G, 6)
            self.kx132_accel.y = round(self.raw_output_data.y * self.CONV_2G, 6)
            self.kx132_accel.z = round(self.raw_output_data.z * self.CONV_2G, 6)
        elif accel_range == self.KX132_RANGE4G:
            self.kx132_accel.x = round(self.raw_output_data.x * self.CONV_4G, 6)
            self.kx132_accel.y = round(self.raw_output_data.y * self.CONV_4G, 6)
            self.kx132_accel.z = round(self.raw_output_data.z * self.CONV_4G, 6)
        elif accel_range == self.KX132_RANGE8G:
            self.kx132_accel.x = round(self.raw_output_data.x * self.CONV_8G, 6)
            self.kx132_accel.y = round(self.raw_output_data.y * self.CONV_8G, 6)
            self.kx132_accel.z = round(self.raw_output_data.z * self.CONV_8G, 6)
        elif accel_range == self.KX132_RANGE16G:
            self.kx132_accel.x = round(self.raw_output_data.x * self.CONV_16G, 6)
            self.kx132_accel.y = round(self.raw_output_data.y * self.CONV_16G, 6)
            self.kx132_accel.z = round(self.raw_output_data.z * self.CONV_16G, 6)


class QwiicKX134(QwiicKX13XCore):

    KX134_WHO_AM_I = 0x46
    KX134_RANGE8G  = 0x00
    KX134_RANGE16G = 0x01
    KX134_RANGE32G = 0x02
    KX134_RANGE64G = 0x03

    CONV_8G =  .000244140751365703299
    CONV_16G = .000488281197546311838
    CONV_32G = .000976523950926236762
    CONV_64G = .001953125095370342112

    kx134_accel = namedtuple('kx134_accel', 'x y z')

    def __init__(self, address = None, i2c_driver = None):
        super().__init__(address, i2c_driver)

    def begin(self):
        """
            Checks that communication can be made with the QwiicKX134 by checking
            the WHO_AM_I register.
            :return: Returns true if WHO_AM_I value is the correct one and
            false otherwise.
            :rtype: bool
        """
        chipID = self.beginCore()
        if chipID == self.KX134_WHO_AM_I:
            return True
        else:
            return False

    def get_accel_data(self):
        """
            Retrieves acceleration data and converts it, storing it within a
            named tuple local to the QwiicKX134 class.
        """
        self.get_raw_accel_data()
        self.conv_accel_data()

    def conv_accel_data(self):
        """
            Converts raw acceleration data according to the range setting and
            stores it in a named tuple local to the QwiicKX132.
        """
        accel_range = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        accel_range &= 0x18
        accel_range = accel_range >> 3

        if accel_range == self.KX134_RANGE8G:
            self.kx134_accel.x = round(self.raw_output_data.x * self.CONV_8G, 6)
            self.kx134_accel.y = round(self.raw_output_data.y * self.CONV_8G, 6)
            self.kx134_accel.z = round(self.raw_output_data.z * self.CONV_8G, 6)
        elif accel_range == self.KX134_RANGE16G:
            self.kx134_accel.x = round(self.raw_output_data.x * self.CONV_16G, 6)
            self.kx134_accel.y = round(self.raw_output_data.y * self.CONV_16G, 6)
            self.kx134_accel.z = round(self.raw_output_data.z * self.CONV_16G, 6)
        elif accel_range == self.KX134_RANGE32G:
            self.kx134_accel.x = round(self.raw_output_data.x * self.CONV_32G, 6)
            self.kx134_accel.y = round(self.raw_output_data.y * self.CONV_32G, 6)
            self.kx134_accel.z = round(self.raw_output_data.z * self.CONV_32G, 6)
        elif accel_range == self.KX134_RANGE64G:
            self.kx134_accel.x = round(self.raw_output_data.x * self.CONV_64G, 6)
            self.kx134_accel.y = round(self.raw_output_data.y * self.CONV_64G, 6)
            self.kx134_accel.z = round(self.raw_output_data.z * self.CONV_64G, 6)

