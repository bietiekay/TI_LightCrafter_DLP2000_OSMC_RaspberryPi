# -*- coding: windows-1252 -*-

# dpp2607.py
#
# sends commands to DPP2607 ASIC using I2C
#
# Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
*** Note *** - this module is generated, changes will be lost!
Python Interface to DLP DPP2607
"""
import time
import struct
from enum import IntEnum
from logging import log, DEBUG
import i2c
COMPOUND_CMD_TIMEOUT = 2.0  # seconds

#####################################################
# Constants
#####################################################


X_0_TO_255_YCRCB = 0
X_16_TO_240_Y_112_TO_112_CRCB = 1
X_1_WHITE_AND_1_BLACK = 9
X_1_WHITE_AND_7_BLACK = 7
X_2_3_VGA_PORTRAIT = 4
X_3_2_VGA_LANDSCAPE = 5
X_4_2_2_YCR_CB_16_BIT = 8
X_4_2_2_YCR_CB_8_BIT = 9
X_90_DEGREE_ROTATION = 1
ACTIVE_HIGH = 1
ACTIVE_HIGH_PDM = 1
ACTIVE_HIGH_PULSE = 1
ACTIVE_LOW = 0
ACTIVE_LOW_PDM = 0
ACTIVE_LOW_PULSE = 0
ANSI_4X4_CHECKERBOARD = 0
BLACK = 0
BLUE = 4
BT_601 = 0
BT_656_I_F = 4
BT_709 = 1
COMPLETE = 1
CYAN = 6
DATA_SAMPLES_ON_FALLING_EDGE = 1
DATA_SAMPLES_ON_RISING_EDGE = 0
DIAGONAL_LINES = 10
DISABLED = 0
DLPC2601 = 130
DLPC2607 = 138
DSYS_PORTA_BIT_0 = 0
DSYS_PORTA_BIT_1 = 1
DSYS_PORTA_BIT_2 = 2
DSYS_PORTA_BIT_3 = 3
DSYS_PORTA_BIT_4 = 4
DSYS_PORTA_BIT_5 = 5
DSYS_PORTA_BIT_6 = 6
DSYS_PORTA_BIT_7 = 7
DSYS_PORTB_BIT_0 = 8
DSYS_PORTB_BIT_1 = 9
DSYS_PORTB_BIT_2 = 10
DSYS_PORTB_BIT_3 = 11
DSYS_PORTB_BIT_4 = 12
DSYS_PORTB_BIT_5 = 13
DSYS_PORTB_BIT_6 = 14
DSYS_PORTB_BIT_7 = 15
DSYS_PORTC_BIT_4 = 16
DSYS_PORTC_BIT_5 = 17
DSYS_PORTC_BIT_6 = 18
DSYS_PORTC_BIT_7 = 19
ENABLED = 1
ENABLED_ACTIVATES_CONTROL_BELOW = 1
ERROR_DETECTED = 0
EXTERNAL_VIDEO_PARALLEL_I_F = 0
FINE_CHECKERBOARD = 13
FLASH_BUSY = 1
GAMMA_CURVE_0 = 0
GAMMA_CURVE_1 = 1
GAMMA_CURVE_2 = 2
GAMMA_CURVE_3 = 3
GAMMA_CURVE_4 = 4
GAMMA_CURVE_5 = 5
GAMMA_CURVE_6 = 6
GREEN = 2
HORIZONTAL_GREY_RAMPS = 12
HORIZONTAL_LINES_1W_1B = 9
HORIZONTAL_LINES_1W_7B = 7
INITIALIZATION_COMPLETE = 0
INTERNAL_TEST_PATTERNS = 1
IN_PROGRESS = 0
MAGENTA = 5
NHD_LANDSCAPE = 27
NHD_PORTRAIT = 26
NOT_COMPLETE = 1
NO_ERRORS = 1
NO_ROTATION = 0
NO_TIMEOUTS = 0
NTSC_LANDSCAPE = 23
OFFSET__0 = 0
OFFSET__16 = 1
OPTICAL_TEST_IMAGE = 9
PAL_LANDSCAPE = 25
PARK_THE_DMD = 1
PIO_CYUSBI2C = 16
PIO_CYUSBSPI = 17
PIO_DEVASYS = 3
PIO_GENERICSERIAL = 7
PIO_MMKUSB = 9
PIO_SERIAL = 4
PIO_TESTER = 6
PIO_USB = 5
PIO_USBHID = 10
PIO_USBI2CPRO = 8
QVGA_LANDSCAPE = 1
QVGA_PORTRAIT = 0
QWVGA_LANDSCAPE = 3
QWVGA_PORTRAIT = 2
RED = 1
RGB565_16_BIT = 0
RGB565_8_BIT = 3
RGB666_16_BIT = 7
RGB666_18_BIT = 1
RGB666_8_BIT = 6
RGB888_16_BIT = 5
RGB888_24_BIT = 2
RGB888_8_BIT = 4
SEQUENCE_0 = 0
SEQUENCE_10 = 10
SEQUENCE_11 = 11
SEQUENCE_12 = 12
SEQUENCE_13 = 13
SEQUENCE_14 = 14
SEQUENCE_15 = 15
SEQUENCE_1 = 1
SEQUENCE_2 = 2
SEQUENCE_3 = 3
SEQUENCE_4 = 4
SEQUENCE_5 = 5
SEQUENCE_6 = 6
SEQUENCE_7 = 7
SEQUENCE_8 = 8
SEQUENCE_9 = 9
SET_AS_OFFSET_OFFSET__128 = 1
SET_AS_SIGNED_OFFSET__0 = 0
SOLID_BLACK = 1
SOLID_BLUE = 4
SOLID_GREEN = 3
SOLID_RED = 5
SOLID_WHITE = 2
SPLASH_IMAGE_0 = 0
SPLASH_IMAGE_1 = 1
SPLASH_IMAGE_2 = 2
SPLASH_IMAGE_3 = 3
SPLASH_SCREEN = 2
TIMEOUT_ERROR_HAS_OCCURRED = 1
UNPARK_THE_DMD = 0
VERTICAL_GREY_RAMPS = 11
VERTICAL_LINES_1W_1B = 8
VERTICAL_LINES_1W_7B = 6
VGA_LANDSCAPE = 7
VGA_PORTRAIT = 6
WHITE = 7
WVGA_720_LANDSCAPE = 9
WVGA_720_PORTRAIT = 8
WVGA_752_LANDSCAPE = 11
WVGA_752_PORTRAIT = 10
WVGA_800_LANDSCAPE = 13
WVGA_800_PORTRAIT = 12
WVGA_852_LANDSCAPE = 15
WVGA_852_PORTRAIT = 14
WVGA_853_LANDSCAPE = 17
WVGA_853_PORTRAIT = 16
WVGA_854_LANDSCAPE = 19
WVGA_854_OR_VGA_OUTPUT = 29
WVGA_854_PORTRAIT = 18
WVGA_864_LANDSCAPE = 21
WVGA_864_PORTRAIT = 20
YELLOW = 3


#####################################################
# Enumerations uses by function parameters
#####################################################


class DMDCurtainColor(IntEnum):
    """
    DMD Curtain Color
    """
    BLACK = 0x00
    RED = 0x01
    GREEN = 0x02
    BLUE = 0x04
    YELLOW = 0x03
    MAGENTA = 0x05
    CYAN = 0x06
    WHITE = 0x07


class TestPatternVLines(IntEnum):
    """
    Line Count
    """
    X_1_WHITE_AND_7_BLACK = 0x06
    X_1_WHITE_AND_1_BLACK = 0x08


class TestPatternHLines(IntEnum):
    """
    Line Count
    """
    X_1_WHITE_AND_7_BLACK = 0x07
    X_1_WHITE_AND_1_BLACK = 0x09


class PolarityPixelClock(IntEnum):
    """
    Pixel Clock Polarity
    """
    DATA_SAMPLES_ON_RISING_EDGE = 0x00
    DATA_SAMPLES_ON_FALLING_EDGE = 0x01


class DevLEDStatus(IntEnum):
    """
    LED Timeout Status
    """
    NO_TIMEOUTS = 0x00
    TIMEOUT_ERROR_HAS_OCCURRED = 0x01


class PixFormat(IntEnum):
    """
    Pixel Data Format
    """
    RGB565_16_BIT_ = 0x00
    RGB666_18_BIT_ = 0x01
    RGB888_24_BIT_ = 0x02
    RGB565_8_BIT_ = 0x03
    RGB888_8_BIT_ = 0x04
    RGB888_16_BIT_ = 0x05
    RGB666_8_BIT_ = 0x06
    RGB666_16_BIT_ = 0x07
    X_4_2_2_YCR_CB_16_BIT_ = 0x08
    X_4_2_2_YCR_CB_8_BIT_ = 0x09


class DMDPARK(IntEnum):
    """
    DMD Park Control
    """
    UNPARK_THE_DMD = 0x00
    PARK_THE_DMD = 0x01


class Resolution(IntEnum):
    """
    Resolution
    """
    QVGA_PORTRAIT = 0x00
    QVGA_LANDSCAPE = 0x01
    QWVGA_PORTRAIT = 0x02
    QWVGA_LANDSCAPE = 0x03
    X_2_3_VGA_PORTRAIT = 0x04
    X_3_2_VGA_LANDSCAPE = 0x05
    VGA_PORTRAIT = 0x06
    VGA_LANDSCAPE = 0x07
    WVGA_720_PORTRAIT = 0x08
    WVGA_720_LANDSCAPE = 0x09
    WVGA_752_PORTRAIT = 0x0A
    WVGA_752_LANDSCAPE = 0x0B
    WVGA_800_PORTRAIT = 0x0C
    WVGA_800_LANDSCAPE = 0x0D
    WVGA_852_PORTRAIT = 0x0E
    WVGA_852_LANDSCAPE = 0x0F
    WVGA_853_PORTRAIT = 0x10
    WVGA_853_LANDSCAPE = 0x11
    WVGA_854_PORTRAIT = 0x12
    WVGA_854_LANDSCAPE = 0x13
    WVGA_864_PORTRAIT = 0x14
    WVGA_864_LANDSCAPE = 0x15
    NTSC_LANDSCAPE = 0x17
    PAL_LANDSCAPE = 0x19
    NHD_PORTRAIT = 0x1A
    NHD_LANDSCAPE = 0x1B
    WVGA_854_OR_VGA_OUTPUT = 0x1D


class CompoundStat(IntEnum):
    """
    LED Calibration State
    mDDR Built-In Self-Test State
    """
    COMPLETE = 0x00
    NOT_COMPLETE = 0x01


class TestPattern(IntEnum):
    """
    Current Pattern
    """
    ANSI_4X4_CHECKERBOARD = 0x00
    SOLID_BLACK = 0x01
    SOLID_WHITE = 0x02
    SOLID_GREEN = 0x03
    SOLID_BLUE = 0x04
    SOLID_RED = 0x05
    VERTICAL_LINES_1W_7B_ = 0x06
    HORIZONTAL_LINES_1W_7B_ = 0x07
    VERTICAL_LINES_1W_1B_ = 0x08
    HORIZONTAL_LINES_1W_1B_ = 0x09
    DIAGONAL_LINES = 0x0A
    VERTICAL_GREY_RAMPS = 0x0B
    HORIZONTAL_GREY_RAMPS = 0x0C
    FINE_CHECKERBOARD = 0x0D


class RotationSetting(IntEnum):
    """
    Rotation Setting
    """
    NO_ROTATION = 0x00
    X_90_DEGREE_ROTATION = 0x01


class PolarityDataEn(IntEnum):
    """
    DATAEN Signal Polarity
    """
    ACTIVE_LOW = 0x00
    ACTIVE_HIGH = 0x01


class TestPatternSolids(IntEnum):
    """
    Color
    """
    BLACK = 0x01
    WHITE = 0x02
    GREEN = 0x03
    BLUE = 0x04
    RED = 0x05


class SourceSel(IntEnum):
    """
    Input Source
    """
    EXTERNAL_VIDEO_PARALLEL_I_F_ = 0x00
    INTERNAL_TEST_PATTERNS = 0x01
    SPLASH_SCREEN = 0x02
    BT_656_I_F = 0x04


class DevID(IntEnum):
    """
    Device ID
    """
    DLPC2601 = 0x82
    DLPC2607 = 0x8A


class DevInitStatus(IntEnum):
    """
    Auto-Initialization Status
    """
    IN_PROGRESS = 0x00
    INITIALIZATION_COMPLETE = 0x01


class CompoundLooks(IntEnum):
    """
    Selected Looks Sequence
    """
    SEQUENCE_0 = 0x00
    SEQUENCE_1 = 0x01
    SEQUENCE_2 = 0x02
    SEQUENCE_3 = 0x03
    SEQUENCE_4 = 0x04
    SEQUENCE_5 = 0x05
    SEQUENCE_6 = 0x06
    SEQUENCE_7 = 0x07
    SEQUENCE_8 = 0x08
    SEQUENCE_9 = 0x09
    SEQUENCE_10 = 0x0a
    SEQUENCE_11 = 0x0b
    SEQUENCE_12 = 0x0c
    SEQUENCE_13 = 0x0d
    SEQUENCE_14 = 0x0e
    SEQUENCE_15 = 0x0f


class EnabledDisabled(IntEnum):
    """
    Blue LED State
    DMD Curtain Control
    DMD Long Side Flip
    DMD Short Side Flip
    Green LED State
    Red LED State
    """
    DISABLED = 0x00
    ENABLED = 0x01


class Polarity(IntEnum):
    """
    HSYNC Signal Polarity
    VSYNC Signal Polarity
    """
    ACTIVE_LOW_PULSE = 0x00
    ACTIVE_HIGH_PULSE = 0x01


class DevFlashStatus(IntEnum):
    """
    Flash Initialization Status
    """
    INITIALIZATION_COMPLETE = 0x00
    FLASH_BUSY = 0x01


class CompoundSplash(IntEnum):
    """
    Splash Screen Select
    """
    SPLASH_IMAGE_0 = 0x00
    SPLASH_IMAGE_1 = 0x01
    SPLASH_IMAGE_2 = 0x02
    SPLASH_IMAGE_3 = 0x03
    OPTICAL_TEST_IMAGE = 0x09


#####################################################
# Support functions
#####################################################


def DPP2607_Open(*args):
    """
    Open I2C interface.
    """
    log(DEBUG, "DPP2607_Open()")
    i2c.initialize()


def DPP2607_Close():
    """
    Close I2C interface
    DPP2607_Close().
    :rtype: None
    """
    log(DEBUG, "DPP2607_Close()")
    i2c.terminate()


def DPP2607_GetIODebug():
    """
    Return the IO debugging status.
    :returns: enable, log_path
    :rtype: tuple[bool, str|None]
    """
    return i2c.get_debug(), None


def DPP2607_SetIODebug(enable, log_path=None):
    """
    Enable/disable logging IO to a log file.  Log_path is ignored.
    :type enable: bool
    :type log_path: str, not used
    :rtype: None
    """
    log(DEBUG, "DPP2607_SetIODebug(%s, %s)", enable, log_path)
    i2c.set_debug(enable)


def DPP2607_GetSlaveAddr():
    """
    Get the I2C slave address (default: 0x36).
    :returns: slave_addr
    :rtype: int
    """
    return i2c.get_slave_address()


def DPP2607_SetSlaveAddr(slave_addr):
    """
    Set the I2C slave address (default: 0x36).
    :type slave_addr: int
    :rtype: None
    """
    if slave_addr != i2c.get_slave_address():
        log(DEBUG, "DPP2607_SetSlaveAddr(%s)", hex(slave_addr))
        i2c.terminate()
        i2c.initialize(slave_addr)


def _poll_complete():
    deadline = time.clock() + COMPOUND_CMD_TIMEOUT
    while time.clock() <= deadline:
        i2c.write([0x15, 0x3A])
        status = i2c.read(4)
        if status[3] == 0:
            break  # bit is zero - complete
    else:
        raise IOError(0, "Timeout waiting for DPP2607 Compound Command Completion")


#####################################################
# ASIC Command Functions
#####################################################


def DPP2607_Read_CcaC1r1Coefficient():
    """
    Reads: CCA C1R1 Coefficient.
    DPP2607_Read_CcaC1r1Coefficient(DWORD &&CCAC1R1).
    :returns: ccac1r1
    :rtype: int
    """
    i2c.write([0x15, 0x5F])
    payload = i2c.read(4)
    ccac1r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC1r1Coefficient: ccac1r1=%r', ccac1r1)
    return ccac1r1


def DPP2607_Read_CcaC1r2Coefficient():
    """
    Reads: CCA C1R2 Coefficient.
    DPP2607_Read_CcaC1r2Coefficient(DWORD &&CCAC1R2).
    :returns: ccac1r2
    :rtype: int
    """
    i2c.write([0x15, 0x60])
    payload = i2c.read(4)
    ccac1r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC1r2Coefficient: ccac1r2=%r', ccac1r2)
    return ccac1r2


def DPP2607_Read_CcaC1r3Coefficient():
    """
    Reads: CCA C1R3 Coefficient.
    DPP2607_Read_CcaC1r3Coefficient(DWORD &&CCAC1R3).
    :returns: ccac1r3
    :rtype: int
    """
    i2c.write([0x15, 0x61])
    payload = i2c.read(4)
    ccac1r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC1r3Coefficient: ccac1r3=%r', ccac1r3)
    return ccac1r3


def DPP2607_Read_CcaC2r1Coefficient():
    """
    Reads: CCA C2R1 Coefficient.
    DPP2607_Read_CcaC2r1Coefficient(DWORD &&CCAC2R1).
    :returns: ccac2r1
    :rtype: int
    """
    i2c.write([0x15, 0x62])
    payload = i2c.read(4)
    ccac2r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC2r1Coefficient: ccac2r1=%r', ccac2r1)
    return ccac2r1


def DPP2607_Read_CcaC2r2Coefficient():
    """
    Reads: CCA C2R2 Coefficient.
    DPP2607_Read_CcaC2r2Coefficient(DWORD &&CCAC2R2).
    :returns: ccac2r2
    :rtype: int
    """
    i2c.write([0x15, 0x63])
    payload = i2c.read(4)
    ccac2r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC2r2Coefficient: ccac2r2=%r', ccac2r2)
    return ccac2r2


def DPP2607_Read_CcaC2r3Coefficient():
    """
    Reads: CCA C2R3 Coefficient.
    DPP2607_Read_CcaC2r3Coefficient(DWORD &&CCAC2R3).
    :returns: ccac2r3
    :rtype: int
    """
    i2c.write([0x15, 0x64])
    payload = i2c.read(4)
    ccac2r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC2r3Coefficient: ccac2r3=%r', ccac2r3)
    return ccac2r3


def DPP2607_Read_CcaC3r1Coefficient():
    """
    Reads: CCA C3R1 Coefficient.
    DPP2607_Read_CcaC3r1Coefficient(DWORD &&CCAC3R1).
    :returns: ccac3r1
    :rtype: int
    """
    i2c.write([0x15, 0x65])
    payload = i2c.read(4)
    ccac3r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC3r1Coefficient: ccac3r1=%r', ccac3r1)
    return ccac3r1


def DPP2607_Read_CcaC3r2Coefficient():
    """
    Reads: CCA C3R2 Coefficient.
    DPP2607_Read_CcaC3r2Coefficient(DWORD &&CCAC3R2).
    :returns: ccac3r2
    :rtype: int
    """
    i2c.write([0x15, 0x66])
    payload = i2c.read(4)
    ccac3r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC3r2Coefficient: ccac3r2=%r', ccac3r2)
    return ccac3r2


def DPP2607_Read_CcaC3r3Coefficient():
    """
    Reads: CCA C3R3 Coefficient.
    DPP2607_Read_CcaC3r3Coefficient(DWORD &&CCAC3R3).
    :returns: ccac3r3
    :rtype: int
    """
    i2c.write([0x15, 0x67])
    payload = i2c.read(4)
    ccac3r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC3r3Coefficient: ccac3r3=%r', ccac3r3)
    return ccac3r3


def DPP2607_Read_CcaC4r1Coefficient():
    """
    Reads: CCA C4R1 Coefficient.
    DPP2607_Read_CcaC4r1Coefficient(DWORD &&CCAC4R1).
    :returns: ccac4r1
    :rtype: int
    """
    i2c.write([0x15, 0x68])
    payload = i2c.read(4)
    ccac4r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC4r1Coefficient: ccac4r1=%r', ccac4r1)
    return ccac4r1


def DPP2607_Read_CcaC4r2Coefficient():
    """
    Reads: CCA C4R2 Coefficient.
    DPP2607_Read_CcaC4r2Coefficient(DWORD &&CCAC4R2).
    :returns: ccac4r2
    :rtype: int
    """
    i2c.write([0x15, 0x69])
    payload = i2c.read(4)
    ccac4r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC4r2Coefficient: ccac4r2=%r', ccac4r2)
    return ccac4r2


def DPP2607_Read_CcaC4r3Coefficient():
    """
    Reads: CCA C4R3 Coefficient.
    DPP2607_Read_CcaC4r3Coefficient(DWORD &&CCAC4R3).
    :returns: ccac4r3
    :rtype: int
    """
    i2c.write([0x15, 0x6A])
    payload = i2c.read(4)
    ccac4r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC4r3Coefficient: ccac4r3=%r', ccac4r3)
    return ccac4r3


def DPP2607_Read_CcaC5r1Coefficient():
    """
    Reads: CCA C5R1 Coefficient.
    DPP2607_Read_CcaC5r1Coefficient(DWORD &&CCAC5R1).
    :returns: ccac5r1
    :rtype: int
    """
    i2c.write([0x15, 0x6B])
    payload = i2c.read(4)
    ccac5r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC5r1Coefficient: ccac5r1=%r', ccac5r1)
    return ccac5r1


def DPP2607_Read_CcaC5r2Coefficient():
    """
    Reads: CCA C5R2 Coefficient.
    DPP2607_Read_CcaC5r2Coefficient(DWORD &&CCAC5R2).
    :returns: ccac5r2
    :rtype: int
    """
    i2c.write([0x15, 0x6C])
    payload = i2c.read(4)
    ccac5r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC5r2Coefficient: ccac5r2=%r', ccac5r2)
    return ccac5r2


def DPP2607_Read_CcaC5r3Coefficient():
    """
    Reads: CCA C5R3 Coefficient.
    DPP2607_Read_CcaC5r3Coefficient(DWORD &&CCAC5R3).
    :returns: ccac5r3
    :rtype: int
    """
    i2c.write([0x15, 0x6D])
    payload = i2c.read(4)
    ccac5r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC5r3Coefficient: ccac5r3=%r', ccac5r3)
    return ccac5r3


def DPP2607_Read_CcaC6r1Coefficient():
    """
    Reads: CCA C6R1 Coefficient.
    DPP2607_Read_CcaC6r1Coefficient(DWORD &&CCAC6R1).
    :returns: ccac6r1
    :rtype: int
    """
    i2c.write([0x15, 0x6E])
    payload = i2c.read(4)
    ccac6r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC6r1Coefficient: ccac6r1=%r', ccac6r1)
    return ccac6r1


def DPP2607_Read_CcaC6r2Coefficient():
    """
    Reads: CCA C6R2 Coefficient.
    DPP2607_Read_CcaC6r2Coefficient(DWORD &&CCAC6R2).
    :returns: ccac6r2
    :rtype: int
    """
    i2c.write([0x15, 0x6F])
    payload = i2c.read(4)
    ccac6r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC6r2Coefficient: ccac6r2=%r', ccac6r2)
    return ccac6r2


def DPP2607_Read_CcaC6r3Coefficient():
    """
    Reads: CCA C6R3 Coefficient.
    DPP2607_Read_CcaC6r3Coefficient(DWORD &&CCAC6R3).
    :returns: ccac6r3
    :rtype: int
    """
    i2c.write([0x15, 0x70])
    payload = i2c.read(4)
    ccac6r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC6r3Coefficient: ccac6r3=%r', ccac6r3)
    return ccac6r3


def DPP2607_Read_CcaC7r1Coefficient():
    """
    Reads: CCA C7R1 Coefficient.
    DPP2607_Read_CcaC7r1Coefficient(DWORD &&CCAC7R1).
    :returns: ccac7r1
    :rtype: int
    """
    i2c.write([0x15, 0x71])
    payload = i2c.read(4)
    ccac7r1 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC7r1Coefficient: ccac7r1=%r', ccac7r1)
    return ccac7r1


def DPP2607_Read_CcaC7r2Coefficient():
    """
    Reads: CCA C7R2 Coefficient.
    DPP2607_Read_CcaC7r2Coefficient(DWORD &&CCAC7R2).
    :returns: ccac7r2
    :rtype: int
    """
    i2c.write([0x15, 0x72])
    payload = i2c.read(4)
    ccac7r2 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC7r2Coefficient: ccac7r2=%r', ccac7r2)
    return ccac7r2


def DPP2607_Read_CcaC7r3Coefficient():
    """
    Reads: CCA C7R3 Coefficient.
    DPP2607_Read_CcaC7r3Coefficient(DWORD &&CCAC7R3).
    :returns: ccac7r3
    :rtype: int
    """
    i2c.write([0x15, 0x73])
    payload = i2c.read(4)
    ccac7r3 = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1ff
    log(DEBUG, 'DPP2607_Read_CcaC7r3Coefficient: ccac7r3=%r', ccac7r3)
    return ccac7r3


def DPP2607_Read_CcaFunctionEnable():
    """
    Reads: CCA Function Enable.
    DPP2607_Read_CcaFunctionEnable(DWORD &&CCAEnable).
    :returns: cca_enable
    :rtype: int
    """
    i2c.write([0x15, 0x5E])
    payload = i2c.read(4)
    cca_enable = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1
    log(DEBUG, 'DPP2607_Read_CcaFunctionEnable: cca_enable=%r', cca_enable)
    return cca_enable


def DPP2607_Read_CommunicationStatus():
    """
    Reads: Communication Status.
    DPP2607_Read_CommunicationStatus(DWORD &&CompoundStatInvCmd, DWORD &&CompoundStatParCmd, DWORD &&CompoundStatMemRd, DWORD &&CompoundStatCmdPar, DWORD &&CompoundStatCmdAbt).
    :returns: compound_stat_inv_cmd, compound_stat_par_cmd, compound_stat_mem_rd, compound_stat_cmd_par, compound_stat_cmd_abt
    :rtype: tuple[int, int, int, int, int]
    """
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xC4])
    _poll_complete()
    i2c.write([0x15, 0x39])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    compound_stat_inv_cmd = (value >> 8) & 0x1
    compound_stat_par_cmd = (value >> 9) & 0x1
    compound_stat_mem_rd = (value >> 10) & 0x1
    compound_stat_cmd_par = (value >> 11) & 0x1
    compound_stat_cmd_abt = (value >> 12) & 0x1
    log(DEBUG, 'DPP2607_Read_CommunicationStatus: compound_stat_inv_cmd=%r, compound_stat_par_cmd=%r, compound_stat_mem_rd=%r, compound_stat_cmd_par=%r, compound_stat_cmd_abt=%r', compound_stat_inv_cmd, compound_stat_par_cmd, compound_stat_mem_rd, compound_stat_cmd_par, compound_stat_cmd_abt)
    return compound_stat_inv_cmd, compound_stat_par_cmd, compound_stat_mem_rd, compound_stat_cmd_par, compound_stat_cmd_abt


def DPP2607_Read_CropFirstLine():
    """
    Reads: Crop - First Line.
    DPP2607_Read_CropFirstLine(DWORD &&FirstActiveLine).
    :returns: first_active_line
    :rtype: int
    """
    i2c.write([0x15, 0x29])
    payload = i2c.read(4)
    first_active_line = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_CropFirstLine: first_active_line=%r', first_active_line)
    return first_active_line


def DPP2607_Read_CropFirstPixel():
    """
    Reads: Crop - First Pixel.
    DPP2607_Read_CropFirstPixel(DWORD &&FirstActivePixel).
    :returns: first_active_pixel
    :rtype: int
    """
    i2c.write([0x15, 0x2B])
    payload = i2c.read(4)
    first_active_pixel = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_CropFirstPixel: first_active_pixel=%r', first_active_pixel)
    return first_active_pixel


def DPP2607_Read_CropLastLine():
    """
    Reads: Crop - Last Line.
    DPP2607_Read_CropLastLine(DWORD &&LastActiveLine).
    :returns: last_active_line
    :rtype: int
    """
    i2c.write([0x15, 0x2A])
    payload = i2c.read(4)
    last_active_line = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_CropLastLine: last_active_line=%r', last_active_line)
    return last_active_line


def DPP2607_Read_CropLastPixel():
    """
    Reads: Crop - Last Pixel.
    DPP2607_Read_CropLastPixel(DWORD &&LastActivePixel).
    :returns: last_active_pixel
    :rtype: int
    """
    i2c.write([0x15, 0x2C])
    payload = i2c.read(4)
    last_active_pixel = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_CropLastPixel: last_active_pixel=%r', last_active_pixel)
    return last_active_pixel


def DPP2607_Read_DeviceStatus():
    """
    Reads: Device Status.
    DPP2607_Read_DeviceStatus(DWORD &&DevID, DWORD &&DevFlashStatus, DWORD &&DevInitStatus, DWORD &&DevLEDStatus).
    :returns: dev_id, dev_flash_status, dev_init_status, dev_led_status
    :rtype: tuple[DevID, DevFlashStatus, DevInitStatus, DevLEDStatus]
    """
    i2c.write([0x15, 0x03])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    dev_id = DevID((value >> 0) & 0xff)
    dev_flash_status = DevFlashStatus((value >> 10) & 0x1)
    dev_init_status = DevInitStatus((value >> 11) & 0x1)
    dev_led_status = DevLEDStatus((value >> 12) & 0x1)
    log(DEBUG, 'DPP2607_Read_DeviceStatus: dev_id=%r, dev_flash_status=%r, dev_init_status=%r, dev_led_status=%r', dev_id, dev_flash_status, dev_init_status, dev_led_status)
    return dev_id, dev_flash_status, dev_init_status, dev_led_status


def DPP2607_Read_DisplayCurtainControl():
    """
    Reads: Display Curtain Control.
    DPP2607_Read_DisplayCurtainControl(DWORD &&DMDCurtainCtl, DWORD &&DMDCurtainColor).
    :returns: dmd_curtain_ctl, dmd_curtain_color
    :rtype: tuple[EnabledDisabled, DMDCurtainColor]
    """
    i2c.write([0x15, 0xA6])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    dmd_curtain_ctl = EnabledDisabled((value >> 0) & 0xf)
    dmd_curtain_color = DMDCurtainColor((value >> 4) & 0xf)
    log(DEBUG, 'DPP2607_Read_DisplayCurtainControl: dmd_curtain_ctl=%r, dmd_curtain_color=%r', dmd_curtain_ctl, dmd_curtain_color)
    return dmd_curtain_ctl, dmd_curtain_color


def DPP2607_Read_DmdPark():
    """
    Reads: DMD PARK.
    DPP2607_Read_DmdPark(DWORD &&DMDPARK).
    :returns: dmdpark
    :rtype: DMDPARK
    """
    i2c.write([0x15, 0x2D])
    payload = i2c.read(4)
    dmdpark = DMDPARK((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1)
    log(DEBUG, 'DPP2607_Read_DmdPark: dmdpark=%r', dmdpark)
    return dmdpark


def DPP2607_Read_EmbeddedSoftwareVersion():
    """
    Reads: Embedded Software Version.
    DPP2607_Read_EmbeddedSoftwareVersion(DWORD &&CompoundICPPatch, DWORD &&CompoundICPMinor, DWORD &&CompoundICPMajor).
    :returns: compound_icp_patch, compound_icp_minor, compound_icp_major
    :rtype: tuple[int, int, int]
    """
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0x02])
    _poll_complete()
    i2c.write([0x15, 0x39])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    compound_icp_patch = (value >> 0) & 0xffff
    compound_icp_minor = (value >> 16) & 0xff
    compound_icp_major = (value >> 24) & 0xff
    log(DEBUG, 'DPP2607_Read_EmbeddedSoftwareVersion: compound_icp_patch=%r, compound_icp_minor=%r, compound_icp_major=%r', compound_icp_patch, compound_icp_minor, compound_icp_major)
    return compound_icp_patch, compound_icp_minor, compound_icp_major


def DPP2607_Read_ImageLongFlip():
    """
    Reads: Image Long Flip.
    DPP2607_Read_ImageLongFlip(DWORD &&FlipLong).
    :returns: flip_long
    :rtype: EnabledDisabled
    """
    i2c.write([0x15, 0x0F])
    payload = i2c.read(4)
    flip_long = EnabledDisabled((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1)
    log(DEBUG, 'DPP2607_Read_ImageLongFlip: flip_long=%r', flip_long)
    return flip_long


def DPP2607_Read_ImageRotationSettings():
    """
    Reads: Image Rotation Settings.
    DPP2607_Read_ImageRotationSettings(DWORD &&RotationSetting).
    :returns: rotation_setting
    :rtype: RotationSetting
    """
    i2c.write([0x15, 0x0E])
    payload = i2c.read(4)
    rotation_setting = RotationSetting((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1)
    log(DEBUG, 'DPP2607_Read_ImageRotationSettings: rotation_setting=%r', rotation_setting)
    return rotation_setting


def DPP2607_Read_ImageShortFlip():
    """
    Reads: Image Short Flip.
    DPP2607_Read_ImageShortFlip(DWORD &&FlipShort).
    :returns: flip_short
    :rtype: EnabledDisabled
    """
    i2c.write([0x15, 0x10])
    payload = i2c.read(4)
    flip_short = EnabledDisabled((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1)
    log(DEBUG, 'DPP2607_Read_ImageShortFlip: flip_short=%r', flip_short)
    return flip_short


def DPP2607_Read_InternalTestPattern():
    """
    Reads: Internal Test Pattern.
    DPP2607_Read_InternalTestPattern(DWORD &&TestPattern).
    :returns: test_pattern
    :rtype: TestPattern
    """
    i2c.write([0x15, 0x11])
    payload = i2c.read(4)
    test_pattern = TestPattern((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0xf)
    log(DEBUG, 'DPP2607_Read_InternalTestPattern: test_pattern=%r', test_pattern)
    return test_pattern


def DPP2607_Read_InterruptStatus():
    """
    Reads: Interrupt Status.
    DPP2607_Read_InterruptStatus(DWORD &&IntSeqAbort, DWORD &&IntDMDResetOverrun, DWORD &&IntDMDBlockError, DWORD &&IntDMDIFOverrun, DWORD &&IntFormatBufOverflow, DWORD &&IntFormatStarvation, DWORD &&IntFlashFIFOErr, DWORD &&IntFlashDMAErr, DWORD &&IntFormatMultErr, DWORD &&IntFormatCmdErr, DWORD &&IntFormatQueueWarn, DWORD &&IntDDROverflowBP, DWORD &&IntDDROverflowFB, DWORD &&IntScalerLineErr, DWORD &&IntScalerPixerr, DWORD &&IntLEDTimeout).
    :returns: int_seq_abort, int_dmd_reset_overrun, int_dmd_block_error, int_dmdif_overrun, int_format_buf_overflow, int_format_starvation, int_flash_fifo_err, int_flash_dma_err, int_format_mult_err, int_format_cmd_err, int_format_queue_warn, int_ddr_overflow_bp, int_ddr_overflow_fb, int_scaler_line_err, int_scaler_pixerr, int_led_timeout
    :rtype: tuple[int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int]
    """
    i2c.write([0x15, 0x00])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    int_seq_abort = (value >> 0) & 0x1
    int_dmd_reset_overrun = (value >> 1) & 0x1
    int_dmd_block_error = (value >> 2) & 0x1
    int_dmdif_overrun = (value >> 3) & 0x1
    int_format_buf_overflow = (value >> 4) & 0x1
    int_format_starvation = (value >> 5) & 0x1
    int_flash_fifo_err = (value >> 7) & 0x1
    int_flash_dma_err = (value >> 8) & 0x1
    int_format_mult_err = (value >> 9) & 0x1
    int_format_cmd_err = (value >> 10) & 0x1
    int_format_queue_warn = (value >> 11) & 0x1
    int_ddr_overflow_bp = (value >> 12) & 0x1
    int_ddr_overflow_fb = (value >> 13) & 0x1
    int_scaler_line_err = (value >> 14) & 0x1
    int_scaler_pixerr = (value >> 15) & 0x1
    int_led_timeout = (value >> 18) & 0x1
    log(DEBUG, 'DPP2607_Read_InterruptStatus: int_seq_abort=%r, int_dmd_reset_overrun=%r, int_dmd_block_error=%r, int_dmdif_overrun=%r, int_format_buf_overflow=%r, int_format_starvation=%r, int_flash_fifo_err=%r, int_flash_dma_err=%r, int_format_mult_err=%r, int_format_cmd_err=%r, int_format_queue_warn=%r, int_ddr_overflow_bp=%r, int_ddr_overflow_fb=%r, int_scaler_line_err=%r, int_scaler_pixerr=%r, int_led_timeout=%r', int_seq_abort, int_dmd_reset_overrun, int_dmd_block_error, int_dmdif_overrun, int_format_buf_overflow, int_format_starvation, int_flash_fifo_err, int_flash_dma_err, int_format_mult_err, int_format_cmd_err, int_format_queue_warn, int_ddr_overflow_bp, int_ddr_overflow_fb, int_scaler_line_err, int_scaler_pixerr, int_led_timeout)
    return int_seq_abort, int_dmd_reset_overrun, int_dmd_block_error, int_dmdif_overrun, int_format_buf_overflow, int_format_starvation, int_flash_fifo_err, int_flash_dma_err, int_format_mult_err, int_format_cmd_err, int_format_queue_warn, int_ddr_overflow_bp, int_ddr_overflow_fb, int_scaler_line_err, int_scaler_pixerr, int_led_timeout


def DPP2607_Read_LedCurrentBlue():
    """
    Reads: LED Current - Blue.
    DPP2607_Read_LedCurrentBlue(DWORD &&PWMBlu).
    :returns: pwm_blu
    :rtype: int
    """
    i2c.write([0x15, 0x14])
    payload = i2c.read(4)
    pwm_blu = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_LedCurrentBlue: pwm_blu=%r', pwm_blu)
    return pwm_blu


def DPP2607_Read_LedCurrentGreen():
    """
    Reads: LED Current - Green.
    DPP2607_Read_LedCurrentGreen(DWORD &&PWMGrn).
    :returns: pwm_grn
    :rtype: int
    """
    i2c.write([0x15, 0x13])
    payload = i2c.read(4)
    pwm_grn = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_LedCurrentGreen: pwm_grn=%r', pwm_grn)
    return pwm_grn


def DPP2607_Read_LedCurrentRed():
    """
    Reads: LED Current - Red.
    DPP2607_Read_LedCurrentRed(DWORD &&PWMRed).
    :returns: pwm_red
    :rtype: int
    """
    i2c.write([0x15, 0x12])
    payload = i2c.read(4)
    pwm_red = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7ff
    log(DEBUG, 'DPP2607_Read_LedCurrentRed: pwm_red=%r', pwm_red)
    return pwm_red


def DPP2607_Read_LedDriverEnable():
    """
    Reads: LED Driver Enable.
    DPP2607_Read_LedDriverEnable(DWORD &&LEDEnableRed, DWORD &&LEDEnableGrn, DWORD &&LEDEnableBlu).
    :returns: led_enable_red, led_enable_grn, led_enable_blu
    :rtype: tuple[EnabledDisabled, EnabledDisabled, EnabledDisabled]
    """
    i2c.write([0x15, 0x16])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    led_enable_red = EnabledDisabled((value >> 0) & 0x1)
    led_enable_grn = EnabledDisabled((value >> 1) & 0x1)
    led_enable_blu = EnabledDisabled((value >> 2) & 0x1)
    log(DEBUG, 'DPP2607_Read_LedDriverEnable: led_enable_red=%r, led_enable_grn=%r, led_enable_blu=%r', led_enable_red, led_enable_grn, led_enable_blu)
    return led_enable_red, led_enable_grn, led_enable_blu


def DPP2607_Read_ParallelBusPolarityControl():
    """
    Reads: Parallel Bus Polarity Control.
    DPP2607_Read_ParallelBusPolarityControl(DWORD &&PolarityHSYNC, DWORD &&PolarityVSYNC, DWORD &&PolarityPixelClock, DWORD &&PolarityDataEn).
    :returns: polarity_hsync, polarity_vsync, polarity_pixel_clock, polarity_data_en
    :rtype: tuple[Polarity, Polarity, PolarityPixelClock, PolarityDataEn]
    """
    i2c.write([0x15, 0xAF])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    polarity_hsync = Polarity((value >> 1) & 0x1)
    polarity_vsync = Polarity((value >> 2) & 0x1)
    polarity_pixel_clock = PolarityPixelClock((value >> 3) & 0x1)
    polarity_data_en = PolarityDataEn((value >> 4) & 0x1)
    log(DEBUG, 'DPP2607_Read_ParallelBusPolarityControl: polarity_hsync=%r, polarity_vsync=%r, polarity_pixel_clock=%r, polarity_data_en=%r', polarity_hsync, polarity_vsync, polarity_pixel_clock, polarity_data_en)
    return polarity_hsync, polarity_vsync, polarity_pixel_clock, polarity_data_en


def DPP2607_Read_SystemStatus():
    """
    Reads: System Status.
    DPP2607_Read_SystemStatus(DWORD &&CompoundStatInit, DWORD &&CompoundStatFlash, DWORD &&CompoundStatTemp, DWORD &&CompoundStatPAD, DWORD &&CompoundStatLED, DWORD &&CompoundStatBIST).
    :returns: compound_stat_init, compound_stat_flash, compound_stat_temp, compound_stat_pad, compound_stat_led, compound_stat_bist
    :rtype: tuple[int, int, int, int, CompoundStat, CompoundStat]
    """
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xC4])
    _poll_complete()
    i2c.write([0x15, 0x39])
    payload = i2c.read(4)
    value = struct.unpack(">I", str(bytearray(payload[0:4])))[0]
    compound_stat_init = (value >> 0) & 0x1
    compound_stat_flash = (value >> 1) & 0x1
    compound_stat_temp = (value >> 2) & 0x1
    compound_stat_pad = (value >> 3) & 0x1
    compound_stat_led = CompoundStat((value >> 5) & 0x1)
    compound_stat_bist = CompoundStat((value >> 6) & 0x1)
    log(DEBUG, 'DPP2607_Read_SystemStatus: compound_stat_init=%r, compound_stat_flash=%r, compound_stat_temp=%r, compound_stat_pad=%r, compound_stat_led=%r, compound_stat_bist=%r', compound_stat_init, compound_stat_flash, compound_stat_temp, compound_stat_pad, compound_stat_led, compound_stat_bist)
    return compound_stat_init, compound_stat_flash, compound_stat_temp, compound_stat_pad, compound_stat_led, compound_stat_bist


def DPP2607_Read_SystemTemperature():
    """
    Reads: System Temperature.
    DPP2607_Read_SystemTemperature(DWORD &&CompoundTemp).
    :returns: compound_temp
    :rtype: int
    """
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xC5])
    _poll_complete()
    i2c.write([0x15, 0x39])
    payload = i2c.read(4)
    compound_temp = (struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0xffffffffL
    log(DEBUG, 'DPP2607_Read_SystemTemperature: compound_temp=%r', compound_temp)
    return compound_temp


def DPP2607_Read_VideoPixelFormat():
    """
    Reads: Video Pixel Format.
    DPP2607_Read_VideoPixelFormat(DWORD &&PixFormat).
    :returns: pix_format
    :rtype: PixFormat
    """
    i2c.write([0x15, 0x0D])
    payload = i2c.read(4)
    pix_format = PixFormat((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0xf)
    log(DEBUG, 'DPP2607_Read_VideoPixelFormat: pix_format=%r', pix_format)
    return pix_format


def DPP2607_Read_VideoResolution():
    """
    Reads: Video Resolution.
    DPP2607_Read_VideoResolution(DWORD &&Resolution).
    :returns: resolution
    :rtype: Resolution
    """
    i2c.write([0x15, 0x0C])
    payload = i2c.read(4)
    resolution = Resolution((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x1f)
    log(DEBUG, 'DPP2607_Read_VideoResolution: resolution=%r', resolution)
    return resolution


def DPP2607_Read_VideoSourceSelection():
    """
    Reads: Video Source Selection.
    DPP2607_Read_VideoSourceSelection(DWORD &&SourceSel).
    :returns: source_sel
    :rtype: SourceSel
    """
    i2c.write([0x15, 0x0B])
    payload = i2c.read(4)
    source_sel = SourceSel((struct.unpack(">I", str(bytearray(payload[0:4])))[0] >> 0) & 0x7)
    log(DEBUG, 'DPP2607_Read_VideoSourceSelection: source_sel=%r', source_sel)
    return source_sel


def DPP2607_Write_CcaC1r1Coefficient(ccac1r1):
    """
    Writes: CCA C1R1 Coefficient.
    DPP2607_Write_CcaC1r1Coefficient(DWORD CCAC1R1).
    :type ccac1r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC1r1Coefficient(%r)', ccac1r1)
    payload = [0x5F]
    payload.extend(list(bytearray(struct.pack(">I", ccac1r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC1r2Coefficient(ccac1r2):
    """
    Writes: CCA C1R2 Coefficient.
    DPP2607_Write_CcaC1r2Coefficient(DWORD CCAC1R2).
    :type ccac1r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC1r2Coefficient(%r)', ccac1r2)
    payload = [0x60]
    payload.extend(list(bytearray(struct.pack(">I", ccac1r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC1r3Coefficient(ccac1r3):
    """
    Writes: CCA C1R3 Coefficient.
    DPP2607_Write_CcaC1r3Coefficient(DWORD CCAC1R3).
    :type ccac1r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC1r3Coefficient(%r)', ccac1r3)
    payload = [0x61]
    payload.extend(list(bytearray(struct.pack(">I", ccac1r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC2r1Coefficient(ccac2r1):
    """
    Writes: CCA C2R1 Coefficient.
    DPP2607_Write_CcaC2r1Coefficient(DWORD CCAC2R1).
    :type ccac2r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC2r1Coefficient(%r)', ccac2r1)
    payload = [0x62]
    payload.extend(list(bytearray(struct.pack(">I", ccac2r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC2r2Coefficient(ccac2r2):
    """
    Writes: CCA C2R2 Coefficient.
    DPP2607_Write_CcaC2r2Coefficient(DWORD CCAC2R2).
    :type ccac2r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC2r2Coefficient(%r)', ccac2r2)
    payload = [0x63]
    payload.extend(list(bytearray(struct.pack(">I", ccac2r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC2r3Coefficient(ccac2r3):
    """
    Writes: CCA C2R3 Coefficient.
    DPP2607_Write_CcaC2r3Coefficient(DWORD CCAC2R3).
    :type ccac2r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC2r3Coefficient(%r)', ccac2r3)
    payload = [0x64]
    payload.extend(list(bytearray(struct.pack(">I", ccac2r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC3r1Coefficient(ccac3r1):
    """
    Writes: CCA C3R1 Coefficient.
    DPP2607_Write_CcaC3r1Coefficient(DWORD CCAC3R1).
    :type ccac3r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC3r1Coefficient(%r)', ccac3r1)
    payload = [0x65]
    payload.extend(list(bytearray(struct.pack(">I", ccac3r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC3r2Coefficient(ccac3r2):
    """
    Writes: CCA C3R2 Coefficient.
    DPP2607_Write_CcaC3r2Coefficient(DWORD CCAC3R2).
    :type ccac3r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC3r2Coefficient(%r)', ccac3r2)
    payload = [0x66]
    payload.extend(list(bytearray(struct.pack(">I", ccac3r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC3r3Coefficient(ccac3r3):
    """
    Writes: CCA C3R3 Coefficient.
    DPP2607_Write_CcaC3r3Coefficient(DWORD CCAC3R3).
    :type ccac3r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC3r3Coefficient(%r)', ccac3r3)
    payload = [0x67]
    payload.extend(list(bytearray(struct.pack(">I", ccac3r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC4r1Coefficient(ccac4r1):
    """
    Writes: CCA C4R1 Coefficient.
    DPP2607_Write_CcaC4r1Coefficient(DWORD CCAC4R1).
    :type ccac4r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC4r1Coefficient(%r)', ccac4r1)
    payload = [0x68]
    payload.extend(list(bytearray(struct.pack(">I", ccac4r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC4r2Coefficient(ccac4r2):
    """
    Writes: CCA C4R2 Coefficient.
    DPP2607_Write_CcaC4r2Coefficient(DWORD CCAC4R2).
    :type ccac4r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC4r2Coefficient(%r)', ccac4r2)
    payload = [0x69]
    payload.extend(list(bytearray(struct.pack(">I", ccac4r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC4r3Coefficient(ccac4r3):
    """
    Writes: CCA C4R3 Coefficient.
    DPP2607_Write_CcaC4r3Coefficient(DWORD CCAC4R3).
    :type ccac4r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC4r3Coefficient(%r)', ccac4r3)
    payload = [0x6A]
    payload.extend(list(bytearray(struct.pack(">I", ccac4r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC5r1Coefficient(ccac5r1):
    """
    Writes: CCA C5R1 Coefficient.
    DPP2607_Write_CcaC5r1Coefficient(DWORD CCAC5R1).
    :type ccac5r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC5r1Coefficient(%r)', ccac5r1)
    payload = [0x6B]
    payload.extend(list(bytearray(struct.pack(">I", ccac5r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC5r2Coefficient(ccac5r2):
    """
    Writes: CCA C5R2 Coefficient.
    DPP2607_Write_CcaC5r2Coefficient(DWORD CCAC5R2).
    :type ccac5r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC5r2Coefficient(%r)', ccac5r2)
    payload = [0x6C]
    payload.extend(list(bytearray(struct.pack(">I", ccac5r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC5r3Coefficient(ccac5r3):
    """
    Writes: CCA C5R3 Coefficient.
    DPP2607_Write_CcaC5r3Coefficient(DWORD CCAC5R3).
    :type ccac5r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC5r3Coefficient(%r)', ccac5r3)
    payload = [0x6D]
    payload.extend(list(bytearray(struct.pack(">I", ccac5r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC6r1Coefficient(ccac6r1):
    """
    Writes: CCA C6R1 Coefficient.
    DPP2607_Write_CcaC6r1Coefficient(DWORD CCAC6R1).
    :type ccac6r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC6r1Coefficient(%r)', ccac6r1)
    payload = [0x6E]
    payload.extend(list(bytearray(struct.pack(">I", ccac6r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC6r2Coefficient(ccac6r2):
    """
    Writes: CCA C6R2 Coefficient.
    DPP2607_Write_CcaC6r2Coefficient(DWORD CCAC6R2).
    :type ccac6r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC6r2Coefficient(%r)', ccac6r2)
    payload = [0x6F]
    payload.extend(list(bytearray(struct.pack(">I", ccac6r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC6r3Coefficient(ccac6r3):
    """
    Writes: CCA C6R3 Coefficient.
    DPP2607_Write_CcaC6r3Coefficient(DWORD CCAC6R3).
    :type ccac6r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC6r3Coefficient(%r)', ccac6r3)
    payload = [0x70]
    payload.extend(list(bytearray(struct.pack(">I", ccac6r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC7r1Coefficient(ccac7r1):
    """
    Writes: CCA C7R1 Coefficient.
    DPP2607_Write_CcaC7r1Coefficient(DWORD CCAC7R1).
    :type ccac7r1: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC7r1Coefficient(%r)', ccac7r1)
    payload = [0x71]
    payload.extend(list(bytearray(struct.pack(">I", ccac7r1 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC7r2Coefficient(ccac7r2):
    """
    Writes: CCA C7R2 Coefficient.
    DPP2607_Write_CcaC7r2Coefficient(DWORD CCAC7R2).
    :type ccac7r2: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC7r2Coefficient(%r)', ccac7r2)
    payload = [0x72]
    payload.extend(list(bytearray(struct.pack(">I", ccac7r2 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaC7r3Coefficient(ccac7r3):
    """
    Writes: CCA C7R3 Coefficient.
    DPP2607_Write_CcaC7r3Coefficient(DWORD CCAC7R3).
    :type ccac7r3: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaC7r3Coefficient(%r)', ccac7r3)
    payload = [0x73]
    payload.extend(list(bytearray(struct.pack(">I", ccac7r3 & 0x1ff))))
    i2c.write(payload)


def DPP2607_Write_CcaFunctionEnable(cca_enable):
    """
    Writes: CCA Function Enable.
    DPP2607_Write_CcaFunctionEnable(DWORD CCAEnable).
    :type cca_enable: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CcaFunctionEnable(%r)', cca_enable)
    payload = [0x5E]
    payload.extend(list(bytearray(struct.pack(">I", cca_enable & 0x1))))
    i2c.write(payload)


def DPP2607_Write_CheckerboardAnsiPattern():
    """
    Writes: Checkerboard ANSI Pattern.
    DPP2607_Write_CheckerboardAnsiPattern().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CheckerboardAnsiPattern()', )
    payload = [0x11]
    payload.extend([0, 0, 0, 13])  # test_pattern_ansi
    i2c.write(payload)


def DPP2607_Write_CropFirstLine(first_active_line):
    """
    Writes: Crop - First Line.
    DPP2607_Write_CropFirstLine(DWORD FirstActiveLine).
    :type first_active_line: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CropFirstLine(%r)', first_active_line)
    payload = [0x29]
    payload.extend(list(bytearray(struct.pack(">I", first_active_line & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_CropFirstPixel(first_active_pixel):
    """
    Writes: Crop - First Pixel.
    DPP2607_Write_CropFirstPixel(DWORD FirstActivePixel).
    :type first_active_pixel: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CropFirstPixel(%r)', first_active_pixel)
    payload = [0x2B]
    payload.extend(list(bytearray(struct.pack(">I", first_active_pixel & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_CropLastLine(last_active_line):
    """
    Writes: Crop - Last Line.
    DPP2607_Write_CropLastLine(DWORD LastActiveLine).
    :type last_active_line: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CropLastLine(%r)', last_active_line)
    payload = [0x2A]
    payload.extend(list(bytearray(struct.pack(">I", last_active_line & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_CropLastPixel(last_active_pixel):
    """
    Writes: Crop - Last Pixel.
    DPP2607_Write_CropLastPixel(DWORD LastActivePixel).
    :type last_active_pixel: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_CropLastPixel(%r)', last_active_pixel)
    payload = [0x2C]
    payload.extend(list(bytearray(struct.pack(">I", last_active_pixel & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_DiagonalLinesPattern():
    """
    Writes: Diagonal Lines Pattern.
    DPP2607_Write_DiagonalLinesPattern().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_DiagonalLinesPattern()', )
    payload = [0x11]
    payload.extend([0, 0, 0, 10])  # test_pattern_d_lines
    i2c.write(payload)


def DPP2607_Write_DisplayCurtainControl(dmd_curtain_ctl, dmd_curtain_color):
    """
    Writes: Display Curtain Control.
    DPP2607_Write_DisplayCurtainControl(DWORD DMDCurtainCtl, DWORD DMDCurtainColor).
    :type dmd_curtain_ctl: EnabledDisabled
    :type dmd_curtain_color: DMDCurtainColor
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_DisplayCurtainControl(%r, %r)', dmd_curtain_ctl, dmd_curtain_color)
    payload = [0xA6]
    value = 0
    value |= (dmd_curtain_ctl & 0xf) << 0
    value |= (dmd_curtain_color & 0xf) << 4
    payload.extend(list(bytearray(struct.pack(">I", value))))
    i2c.write(payload)


def DPP2607_Write_DmdPark(dmdpark):
    """
    Writes: DMD PARK.
    DPP2607_Write_DmdPark(DWORD DMDPARK).
    :type dmdpark: DMDPARK
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_DmdPark(%r)', dmdpark)
    payload = [0x2D]
    payload.extend(list(bytearray(struct.pack(">I", dmdpark & 0x1))))
    i2c.write(payload)


def DPP2607_Write_FineCheckerboardPattern():
    """
    Writes: Fine Checkerboard Pattern.
    DPP2607_Write_FineCheckerboardPattern().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_FineCheckerboardPattern()', )
    payload = [0x11]
    payload.extend([0, 0, 0, 0])  # test_pattern_fine_checker
    i2c.write(payload)


def DPP2607_Write_HorizontalGrayRampPattern():
    """
    Writes: Horizontal Gray Ramp Pattern.
    DPP2607_Write_HorizontalGrayRampPattern().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_HorizontalGrayRampPattern()', )
    payload = [0x11]
    payload.extend([0, 0, 0, 12])  # test_pattern_gray_ramp_h
    i2c.write(payload)


def DPP2607_Write_HorizontalLinesPattern(test_pattern_h_lines):
    """
    Writes: Horizontal Lines Pattern.
    DPP2607_Write_HorizontalLinesPattern(DWORD TestPatternHLines).
    :type test_pattern_h_lines: TestPatternHLines
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_HorizontalLinesPattern(%r)', test_pattern_h_lines)
    payload = [0x11]
    payload.extend(list(bytearray(struct.pack(">I", test_pattern_h_lines & 0xf))))
    i2c.write(payload)


def DPP2607_Write_ImageLongFlip(flip_long):
    """
    Writes: Image Long Flip.
    DPP2607_Write_ImageLongFlip(DWORD FlipLong).
    :type flip_long: EnabledDisabled
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_ImageLongFlip(%r)', flip_long)
    payload = [0x0F]
    payload.extend(list(bytearray(struct.pack(">I", flip_long & 0x1))))
    i2c.write(payload)


def DPP2607_Write_ImageRotationSettings(rotation_setting):
    """
    Writes: Image Rotation Settings.
    DPP2607_Write_ImageRotationSettings(DWORD RotationSetting).
    :type rotation_setting: RotationSetting
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_ImageRotationSettings(%r)', rotation_setting)
    payload = [0x0E]
    payload.extend(list(bytearray(struct.pack(">I", rotation_setting & 0x1))))
    i2c.write(payload)


def DPP2607_Write_ImageShortFlip(flip_short):
    """
    Writes: Image Short Flip.
    DPP2607_Write_ImageShortFlip(DWORD FlipShort).
    :type flip_short: EnabledDisabled
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_ImageShortFlip(%r)', flip_short)
    payload = [0x10]
    payload.extend(list(bytearray(struct.pack(">I", flip_short & 0x1))))
    i2c.write(payload)


def DPP2607_Write_InterruptStatus(int_seq_abort, int_dmd_reset_overrun, int_dmd_block_error, int_dmdif_overrun, int_format_buf_overflow, int_format_starvation, int_flash_fifo_err, int_flash_dma_err, int_format_mult_err, int_format_cmd_err, int_format_queue_warn, int_ddr_overflow_bp, int_ddr_overflow_fb, int_scaler_line_err, int_scaler_pixerr, int_led_timeout):
    """
    Writes: Interrupt Status.
    DPP2607_Write_InterruptStatus(DWORD IntSeqAbort, DWORD IntDMDResetOverrun, DWORD IntDMDBlockError, DWORD IntDMDIFOverrun, DWORD IntFormatBufOverflow, DWORD IntFormatStarvation, DWORD IntFlashFIFOErr, DWORD IntFlashDMAErr, DWORD IntFormatMultErr, DWORD IntFormatCmdErr, DWORD IntFormatQueueWarn, DWORD IntDDROverflowBP, DWORD IntDDROverflowFB, DWORD IntScalerLineErr, DWORD IntScalerPixerr, DWORD IntLEDTimeout).
    :type int_seq_abort: int
    :type int_dmd_reset_overrun: int
    :type int_dmd_block_error: int
    :type int_dmdif_overrun: int
    :type int_format_buf_overflow: int
    :type int_format_starvation: int
    :type int_flash_fifo_err: int
    :type int_flash_dma_err: int
    :type int_format_mult_err: int
    :type int_format_cmd_err: int
    :type int_format_queue_warn: int
    :type int_ddr_overflow_bp: int
    :type int_ddr_overflow_fb: int
    :type int_scaler_line_err: int
    :type int_scaler_pixerr: int
    :type int_led_timeout: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_InterruptStatus(%r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r, %r)', int_seq_abort, int_dmd_reset_overrun, int_dmd_block_error, int_dmdif_overrun, int_format_buf_overflow, int_format_starvation, int_flash_fifo_err, int_flash_dma_err, int_format_mult_err, int_format_cmd_err, int_format_queue_warn, int_ddr_overflow_bp, int_ddr_overflow_fb, int_scaler_line_err, int_scaler_pixerr, int_led_timeout)
    payload = [0x00]
    value = 0
    value |= (int_seq_abort & 0x1) << 0
    value |= (int_dmd_reset_overrun & 0x1) << 1
    value |= (int_dmd_block_error & 0x1) << 2
    value |= (int_dmdif_overrun & 0x1) << 3
    value |= (int_format_buf_overflow & 0x1) << 4
    value |= (int_format_starvation & 0x1) << 5
    value |= (int_flash_fifo_err & 0x1) << 7
    value |= (int_flash_dma_err & 0x1) << 8
    value |= (int_format_mult_err & 0x1) << 9
    value |= (int_format_cmd_err & 0x1) << 10
    value |= (int_format_queue_warn & 0x1) << 11
    value |= (int_ddr_overflow_bp & 0x1) << 12
    value |= (int_ddr_overflow_fb & 0x1) << 13
    value |= (int_scaler_line_err & 0x1) << 14
    value |= (int_scaler_pixerr & 0x1) << 15
    value |= (int_led_timeout & 0x1) << 18
    payload.extend(list(bytearray(struct.pack(">I", value))))
    i2c.write(payload)


def DPP2607_Write_LedCurrentBlue(pwm_blu):
    """
    Writes: LED Current - Blue.
    DPP2607_Write_LedCurrentBlue(DWORD PWMBlu).
    :type pwm_blu: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_LedCurrentBlue(%r)', pwm_blu)
    payload = [0x14]
    payload.extend(list(bytearray(struct.pack(">I", pwm_blu & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_LedCurrentGreen(pwm_grn):
    """
    Writes: LED Current - Green.
    DPP2607_Write_LedCurrentGreen(DWORD PWMGrn).
    :type pwm_grn: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_LedCurrentGreen(%r)', pwm_grn)
    payload = [0x13]
    payload.extend(list(bytearray(struct.pack(">I", pwm_grn & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_LedCurrentRed(pwm_red):
    """
    Writes: LED Current - Red.
    DPP2607_Write_LedCurrentRed(DWORD PWMRed).
    :type pwm_red: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_LedCurrentRed(%r)', pwm_red)
    payload = [0x12]
    payload.extend(list(bytearray(struct.pack(">I", pwm_red & 0x7ff))))
    i2c.write(payload)


def DPP2607_Write_LedDriverEnable(led_enable_red, led_enable_grn, led_enable_blu):
    """
    Writes: LED Driver Enable.
    DPP2607_Write_LedDriverEnable(DWORD LEDEnableRed, DWORD LEDEnableGrn, DWORD LEDEnableBlu).
    :type led_enable_red: EnabledDisabled
    :type led_enable_grn: EnabledDisabled
    :type led_enable_blu: EnabledDisabled
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_LedDriverEnable(%r, %r, %r)', led_enable_red, led_enable_grn, led_enable_blu)
    payload = [0x16]
    value = 0
    value |= (led_enable_red & 0x1) << 0
    value |= (led_enable_grn & 0x1) << 1
    value |= (led_enable_blu & 0x1) << 2
    payload.extend(list(bytearray(struct.pack(">I", value))))
    i2c.write(payload)


def DPP2607_Write_ParallelBusPolarityControl(polarity_hsync, polarity_vsync, polarity_pixel_clock, polarity_data_en):
    """
    Writes: Parallel Bus Polarity Control.
    DPP2607_Write_ParallelBusPolarityControl(DWORD PolarityHSYNC, DWORD PolarityVSYNC, DWORD PolarityPixelClock, DWORD PolarityDataEn).
    :type polarity_hsync: Polarity
    :type polarity_vsync: Polarity
    :type polarity_pixel_clock: PolarityPixelClock
    :type polarity_data_en: PolarityDataEn
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_ParallelBusPolarityControl(%r, %r, %r, %r)', polarity_hsync, polarity_vsync, polarity_pixel_clock, polarity_data_en)
    payload = [0xAF]
    value = 0
    value |= (polarity_hsync & 0x1) << 1
    value |= (polarity_vsync & 0x1) << 2
    value |= (polarity_pixel_clock & 0x1) << 3
    value |= (polarity_data_en & 0x1) << 4
    payload.extend(list(bytearray(struct.pack(">I", value))))
    i2c.write(payload)


def DPP2607_Write_PropagateLedCurrents(led_latch):
    """
    Writes: Propagate LED Currents.
    DPP2607_Write_PropagateLedCurrents(DWORD LEDLatch).
    :type led_latch: int
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_PropagateLedCurrents(%r)', led_latch)
    payload = [0x39]
    payload.extend(list(bytearray(struct.pack(">I", led_latch))))
    i2c.write(payload)
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xD3])
    _poll_complete()


def DPP2607_Write_SequenceSelect(compound_looks):
    """
    Writes: Sequence Select.
    DPP2607_Write_SequenceSelect(DWORD CompoundLooks).
    :type compound_looks: CompoundLooks
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_SequenceSelect(%r)', compound_looks)
    payload = [0x39]
    payload.extend(list(bytearray(struct.pack(">I", compound_looks))))
    i2c.write(payload)
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xC1])
    _poll_complete()


def DPP2607_Write_SetSplashScreen(compound_splash):
    """
    Writes: Set Splash Screen.
    DPP2607_Write_SetSplashScreen(DWORD CompoundSplash).
    :type compound_splash: CompoundSplash
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_SetSplashScreen(%r)', compound_splash)
    payload = [0x39]
    payload.extend(list(bytearray(struct.pack(">I", compound_splash))))
    i2c.write(payload)
    i2c.write([0x3A, 0x00, 0x00, 0x00, 0x01])
    i2c.write([0x38, 0x00, 0x00, 0x00, 0xBD])
    _poll_complete()


def DPP2607_Write_SolidFieldPattern(test_pattern_solids):
    """
    Writes: Solid Field Pattern.
    DPP2607_Write_SolidFieldPattern(DWORD TestPatternSolids).
    :type test_pattern_solids: TestPatternSolids
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_SolidFieldPattern(%r)', test_pattern_solids)
    payload = [0x11]
    payload.extend(list(bytearray(struct.pack(">I", test_pattern_solids & 0xf))))
    i2c.write(payload)


def DPP2607_Write_SystemReset():
    """
    Writes: System Reset.
    DPP2607_Write_SystemReset().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_SystemReset()', )
    payload = [0x1F]
    payload.extend([0, 0, 0, 1])  # dev_rst
    i2c.write(payload)


def DPP2607_Write_VeritcalLinesPattern(test_pattern_v_lines):
    """
    Writes: Veritcal Lines Pattern.
    DPP2607_Write_VeritcalLinesPattern(DWORD TestPatternVLines).
    :type test_pattern_v_lines: TestPatternVLines
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_VeritcalLinesPattern(%r)', test_pattern_v_lines)
    payload = [0x11]
    payload.extend(list(bytearray(struct.pack(">I", test_pattern_v_lines & 0xf))))
    i2c.write(payload)


def DPP2607_Write_VerticalGrayRampPattern():
    """
    Writes: Vertical Gray Ramp Pattern.
    DPP2607_Write_VerticalGrayRampPattern().
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_VerticalGrayRampPattern()', )
    payload = [0x11]
    payload.extend([0, 0, 0, 11])  # test_pattern_gray_ramp_v
    i2c.write(payload)


def DPP2607_Write_VideoPixelFormat(pix_format):
    """
    Writes: Video Pixel Format.
    DPP2607_Write_VideoPixelFormat(DWORD PixFormat).
    :type pix_format: PixFormat
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_VideoPixelFormat(%r)', pix_format)
    payload = [0x0D]
    payload.extend(list(bytearray(struct.pack(">I", pix_format & 0xf))))
    i2c.write(payload)


def DPP2607_Write_VideoResolution(resolution):
    """
    Writes: Video Resolution.
    DPP2607_Write_VideoResolution(DWORD Resolution).
    :type resolution: Resolution
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_VideoResolution(%r)', resolution)
    payload = [0x0C]
    payload.extend(list(bytearray(struct.pack(">I", resolution & 0x1f))))
    i2c.write(payload)


def DPP2607_Write_VideoSourceSelection(source_sel):
    """
    Writes: Video Source Selection.
    DPP2607_Write_VideoSourceSelection(DWORD SourceSel).
    :type source_sel: SourceSel
    :rtype: None
    """
    log(DEBUG, 'DPP2607_Write_VideoSourceSelection(%r)', source_sel)
    payload = [0x0B]
    payload.extend(list(bytearray(struct.pack(">I", source_sel & 0x7))))
    i2c.write(payload)
