# -*- coding: utf-8 -*-
#   AndoriDus - A Python wrapper for Andor's scientific cameras
#
#   Original code by
#   Copyright (C) 2009  Hamid Ohadi
#
#   Adapted for iDus, qtlab and Windows XP
#   2010 Martijn Schaafsma
#
#   Reworked by Simon Dickreuter 2016
#
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''
This module offers basic functionality for the Andor Cameras
'''

# Modules for Andor functionality
import platform
import sys
import time
from ctypes import *
from PIL import Image
from .errorcodes import ERROR_CODE


class Camera():
    """
    Camera class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK. Extensive documentation
    on the functions used and error codes can be
    found in the Andor SDK Users Guide
    """

    def __init__(self):
        '''
        Loads and initializes the hardware driver.
        Initializes local parameters
        '''

        # Check operating system and load library
        # for Windows
        if platform.system() == "Windows":
            if platform.architecture()[0] == "64bit":
                self.init_path = "C:\\Program Files\\Andor SOLIS\\"
                self._dll = cdll.LoadLibrary("C:\\Program Files\\Andor SOLIS\\atmcd64d_legacy")
            else:
                raise RuntimeError("Only 64bit Version is supported")
        # for Linux
        elif platform.system() == "Linux":
            self.init_path = "/usr/local/etc/andor"
            dllname = "/usr/local/lib/libandor.so"
            self._dll = cdll.LoadLibrary(dllname)
        else:
            raise RuntimeError("Cannot detect operating system, will now stop")

        self._verbosity = True

        # Initialize the device
        error = self.Initialize(self.init_path)
        print("Initializing: %s" % (ERROR_CODE[error]))

        cw = c_int()
        ch = c_int()
        self._dll.GetDetector(byref(cw), byref(ch))

        # Initiate parameters
        self._width        = cw.value
        self._height       = ch.value
        self._temperature  = None
        self._set_T        = None
        self._gain         = None
        self._gainRange    = None
        self._status       = ERROR_CODE[error]
        self._verbosity    = True
        self._preampgain   = None
        self._channel      = None
        self._outamp       = None
        self._hsspeed      = None
        self._vsspeed      = None
        self._serial       = None
        self._exposure     = None
        self._accumulate   = None
        self._kinetic      = None
        self._bitDepths    = []
        self._preAmpGain   = []
        self._VSSpeeds     = []
        self._noGains      = None
        self._imageArray   = []
        self._noVSSpeeds   = None
        self._HSSpeeds     = []
        self._noADChannels = None
        self._noHSSpeeds   = None
        self._ReadMode     = None


    def __del__(self):
        self.SetTemperature(-10)
        if self.cooler :
            warm = False
            while not warm:
                time.sleep(0.5)
                temp = self.GetTemperature
                if temp > -13:
                    warm = True
            self.CoolerOFF()
        error = self._dll.ShutDown()
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)

    def verbose(self, error, function=''):
        if self._verbosity is True:
            print("[%s]: %s" % (function, error))

    def SetVerbose(self, state=True):
        self._verbosity = state

    def AbortAcquisition(self):
        error = self._dll.AbortAcquisition()
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def Initialize(self, path):
        error = self._dll.Initialize(path)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return error

    def ShutDown(self):
        error = self._dll.ShutDown()
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetCameraSerialNumber(self):
        serial = c_int()
        error = self._dll.GetCameraSerialNumber(byref(serial))
        self.serial = serial.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetReadMode(self, mode):
        # 0: Full vertical binning
        # 1: multi track
        # 2: random track
        # 3: single track
        # 4: image
        error = self._dll.SetReadMode(mode)
        self.ReadMode = mode
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetAcquisitionMode(self, mode):
        # 1: Single scan
        # 3: Kinetic scan
        error = self._dll.SetAcquisitionMode(mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.AcquisitionMode = mode
        return ERROR_CODE[error]

    def SetNumberKinetics(self, numKin):
        error = self._dll.SetNumberKinetics(numKin)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.scans = numKin
        return ERROR_CODE[error]

    def SetNumberAccumulations(self, number):
        error = self._dll.SetNumberAccumulations(number)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetAccumulationCycleTime(self, time):
        error = self._dll.SetAccumulationCycleTime(c_float(time))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetKineticCycleTime(self, time):
        error = self._dll.SetKineticCycleTime(c_float(time))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetShutter(self, typ, mode, closingtime, openingtime):
        error = self._dll.SetShutter(typ, mode, closingtime, openingtime)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetImage(self, hbin, vbin, hstart, hend, vstart, vend):
        self.hbin = hbin
        self.vbin = vbin
        self.hstart = hstart
        self.hend = hend
        self.vstart = vstart
        self.vend = vend

        error = self._dll.SetImage(hbin, vbin, hstart, hend, vstart, vend)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def StartAcquisition(self):
        error = self._dll.StartAcquisition()
        self._dll.WaitForAcquisition()
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetAcquiredData(self, imageArray):
        if (self.ReadMode == 4):
            if (self.AcquisitionMode == 1):
                dim = self.width * self.height / self.hbin / self.vbin
            elif (self.AcquisitionMode == 3):
                dim = self.width * self.height / self.hbin / self.vbin * self.scans
        elif (self.ReadMode == 3 or self.ReadMode == 0):
            if (self.AcquisitionMode == 1):
                dim = self.width
            elif (self.AcquisitionMode == 3):
                dim = self.width * self.scans

        cimageArray = c_int * dim
        cimage = cimageArray()
        error = self._dll.GetAcquiredData(pointer(cimage), dim)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)

        for i in range(len(cimage)):
            imageArray.append(cimage[i])

        self.imageArray = imageArray[:]
        # self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetExposureTime(self, time):
        error = self._dll.SetExposureTime(c_float(time))
        self.exposure = time
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetAcquisitionTimings(self):
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        error = self._dll.GetAcquisitionTimings(byref(exposure), byref(accumulate), byref(kinetic))
        self.exposure = exposure.value
        self.accumulate = accumulate.value
        self.kinetic = kinetic.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetSingleScan(self):
        self.SetReadMode(4)
        self.SetAcquisitionMode(1)
        self.SetImage(1, 1, 1, self.width, 1, self.height)

    def SetCoolerMode(self, mode):
        error = self._dll.SetCoolerMode(mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SaveAsBmp(self, path):
        im = Image.new("RGB", (self.width, self.height), "white")
        pix = im.load()

        for i in range(len(self.imageArray)):
            (row, col) = divmod(i, self.width)
            picvalue = int(round(self.imageArray[i] * 255.0 / 65535))
            pix[col, row] = (picvalue, picvalue, picvalue)

        im.save(path, "BMP")

    def SaveAsTxt(self, path):
        file = open(path, 'w')

        for line in self.imageArray:
            file.write("%g\n" % line)

        file.close()

    def SetImageRotate(self, iRotate):
        error = self._dll.SetImageRotate(iRotate)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)

    def SaveAsBmpNormalised(self, path):

        im = Image.new("RGB", (self.width, self.height), "white")
        pix = im.load()

        maxIntensity = max(self.imageArray)

        for i in range(len(self.imageArray)):
            (row, col) = divmod(i, self.width)
            picvalue = int(round(self.imageArray[i] * 255.0 / maxIntensity))
            pix[col, row] = (picvalue, picvalue, picvalue)

        im.save(path, "BMP")

    def SaveAsFITS(self, filename, type):
        error = self._dll.SaveAsFITS(filename, type)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def CoolerON(self):
        error = self._dll.CoolerON()
        self.cooler = 1
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def CoolerOFF(self):
        error = self._dll.CoolerOFF()
        self.cooler = 0
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def IsCoolerOn(self):
        iCoolerStatus = c_int()
        self.cooler = iCoolerStatus
        error = self._dll.IsCoolerOn(byref(iCoolerStatus))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return iCoolerStatus.value

    def GetTemperature(self):
        ctemperature = c_int()
        error = self._dll.GetTemperature(byref(ctemperature))
        self.temperature = ctemperature.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetTemperature(self, temperature):
        # ctemperature = c_int(temperature)
        # error = self.dll.SetTemperature(byref(ctemperature))
        error = self._dll.SetTemperature(temperature)
        self.set_T = temperature
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetEMCCDGain(self):
        gain = c_int()
        error = self._dll.GetEMCCDGain(byref(gain))
        self.gain = gain.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetEMCCDGainMode(self, gainMode):
        error = self._dll.SetEMCCDGainMode(gainMode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetEMCCDGain(self, gain):
        error = self._dll.SetEMCCDGain(gain)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetEMAdvanced(self, gainAdvanced):
        error = self._dll.SetEMAdvanced(gainAdvanced)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetEMGainRange(self):
        low = c_int()
        high = c_int()
        error = self._dll.GetEMGainRange(byref(low), byref(high))
        self.gainRange = (low.value, high.value)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetNumberADChannels(self):
        noADChannels = c_int()
        error = self._dll.GetNumberADChannels(byref(noADChannels))
        self.noADChannels = noADChannels.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetBitDepth(self):
        bitDepth = c_int()

        self.bitDepths = []

        for i in range(self.noADChannels):
            self._dll.GetBitDepth(i, byref(bitDepth))
            self.bitDepths.append(bitDepth.value)

    def SetADChannel(self, index):
        error = self._dll.SetADChannel(index)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.channel = index
        return ERROR_CODE[error]

    def SetOutputAmplifier(self, index):
        error = self._dll.SetOutputAmplifier(index)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.outamp = index
        return ERROR_CODE[error]

    def GetNumberHSSpeeds(self):
        noHSSpeeds = c_int()
        error = self._dll.GetNumberHSSpeeds(self.channel, self.outamp, byref(noHSSpeeds))
        self.noHSSpeeds = noHSSpeeds.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetHSSpeed(self):
        HSSpeed = c_float()

        self.HSSpeeds = []

        for i in range(self.noHSSpeeds):
            self._dll.GetHSSpeed(self.channel, self.outamp, i, byref(HSSpeed))
            self._HSSpeeds.append(HSSpeed.value)

    def SetHSSpeed(self, itype, index):
        error = self._dll.SetHSSpeed(itype, index)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.hsspeed = index
        return ERROR_CODE[error]

    def GetNumberVSSpeeds(self):
        noVSSpeeds = c_int()
        error = self._dll.GetNumberVSSpeeds(byref(noVSSpeeds))
        self.noVSSpeeds = noVSSpeeds.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetVSSpeed(self):
        VSSpeed = c_float()

        self.VSSpeeds = []

        for i in range(self.noVSSpeeds):
            self._dll.GetVSSpeed(i, byref(VSSpeed))
            self.preVSpeeds.append(VSSpeed.value)

    def SetVSSpeed(self, index):
        error = self._dll.SetVSSpeed(index)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.vsspeed = index
        return ERROR_CODE[error]

    def GetNumberPreAmpGains(self):
        noGains = c_int()
        error = self._dll.GetNumberPreAmpGains(byref(noGains))
        self.noGains = noGains.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetPreAmpGain(self):
        gain = c_float()

        self.preAmpGain = []

        for i in range(self.noGains):
            self._dll.GetPreAmpGain(i, byref(gain))
            self.preAmpGain.append(gain.value)

    def SetPreAmpGain(self, index):
        error = self._dll.SetPreAmpGain(index)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.preampgain = index
        return ERROR_CODE[error]

    def SetTriggerMode(self, mode):
        error = self._dll.SetTriggerMode(mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetStatus(self):
        status = c_int()
        error = self._dll.GetStatus(byref(status))
        self.status = ERROR_CODE[status.value]
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return self.status

    def GetSeriesProgress(self):
        acc = c_long()
        series = c_long()
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return series.value
        else:
            return None

    def GetAccumulationProgress(self):
        acc = c_long()
        series = c_long()
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return acc.value
        else:
            return None

    def SetFrameTransferMode(self, frameTransfer):
        error = self._dll.SetFrameTransferMode(frameTransfer)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetShutterEx(self, typ, mode, closingtime, openingtime, extmode):
        error = self._dll.SetShutterEx(typ, mode, closingtime, openingtime, extmode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetSpool(self, active, method, path, framebuffersize):
        error = self._dll.SetSpool(active, method, c_char_p(path), framebuffersize)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetSingleTrack(self, centre, height):
        error = self._dll.SetSingleTrack(centre, height)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetDemoReady(self):
        error = self.SetSingleScan()
        error = self.SetTriggerMode(0)
        error = self.SetShutter(1, 0, 30, 30)
        error = self.SetExposureTime(0.01)
        return error

    def SetBinning(self, binningmode):
        if (binningmode == 1):
            self.SetImage(1, 1, 1, self.width, 1, self.height)
        elif (binningmode == 2):
            self.SetImage(2, 2, 1, self.width, 1, self.height)
        elif (binningmode == 4):
            self.SetImage(4, 4, 1, self.width, 1, self.height)
        else:
            self.verbose("Binning mode not found")

    def qChange(self, pvname=None, value=None, char_value=None):
        self.qVal = value
        if self.qVal > 25:
            self.GetEMCCDGain()
            if self.gain > 1:
                self.SetEMCCDGain(1)
                print('Charge above 25 pC, setting gain to 1')

