#!/usr/bin/env python
#-----------------------------------------------------------------------------
# ex1.py
#
# Simple Example for the Qwiic BME280 Device
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, April 2021
# 
# This python library supports the SparkFun Electroncis qwiic 
# qwiic sensor/board ecosystem on a Raspberry Pi (and compatable) single
# board computers. 
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2021 SparkFun Electronics
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
# Example 1
#

from __future__ import print_function
import qwiic_kx13x
import time
import sys

def runExample():

        print("\nSparkFun KX13X Accelerometer Example 1\n")
        mySensor = qwiic_kx13x.QwiicKX132()

        if mySensor.connected == False:
                print("The Qwiic KX13X Accelerometer device isn't connected to the system. Please check your connection", \
                        file=sys.stderr)
                return

        if mySensor.begin():
            print("Confirmed can communicate.")
        mySensor.initialize()

        while True:
                
            mySensor.get_accel_data()
            print("X: {0}g Y: {1}g Z: {2}g".format(mySensor.kx132_accel.x,
                                                   mySensor.kx132_accel.y,
                                                   mySensor.kx132_accel.z))
            time.sleep(.2)


if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)
