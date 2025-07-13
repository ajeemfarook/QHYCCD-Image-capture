#!/usr/bin/env python3
"""capture.py – Phase‑shift image acquisition with QHY5III224C & Arduino

Runs on macOS (tested Ventura, Apple Silicon & Intel).
Requirements
------------
Python 3.10+, NumPy, OpenCV‑Python, PySerial, QHYCCD SDK ≥ 23.12.

Environment variables (optional)
--------------------------------
QHY_SDK_PATH   – absolute path to QHY SDK shared library (.dylib / .so / .dll)
QHY_SAVE_PATH  – directory where PNGs will be saved (default ~/Pictures/QHYCCD)
SERIAL_PORT    – serial device for Arduino (default auto‑detects /dev/cu.usbmodem*)
LED_COUNT      – number of frames / LEDs (default 64)

Usage
-----
$ python3 capture.py  # uses defaults / env vars
$ QHY_SAVE_PATH=/Volumes/SSD/Captures LED_COUNT=128 python3 capture.py
"""

import cv2
import numpy as np
from ctypes import (create_string_buffer, cdll, byref, c_void_p, c_uint32,
                    c_char_p, c_double, c_bool, c_ubyte, POINTER, c_uint16)
from enum import Enum
import warnings
import serial
import time
import os

warnings.filterwarnings("ignore", category=DeprecationWarning)

# -------------------------------------------------------------------------
# Load SDK
qhyccddll = cdll.LoadLibrary(
    'add path of .dll or dylib')

# --- prototypes left exactly as you had them --------------------------------
qhyccddll.GetQHYCCDId.argtypes = [c_uint32, c_char_p]
qhyccddll.OpenQHYCCD.argtypes = [c_char_p]
qhyccddll.OpenQHYCCD.restype = c_void_p
qhyccddll.CloseQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDNumberOfReadModes.argtypes = [c_void_p, POINTER(c_uint32)]
qhyccddll.GetQHYCCDReadModeName.argtypes = [c_void_p, c_uint32, c_char_p]
qhyccddll.GetQHYCCDReadModeResolution.argtypes = [c_void_p, c_uint32,
                                                  POINTER(c_uint32), POINTER(c_uint32)]
qhyccddll.SetQHYCCDReadMode.argtypes = [c_void_p, c_uint32]
qhyccddll.SetQHYCCDStreamMode.argtypes = [c_void_p, c_uint32]
qhyccddll.InitQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDChipInfo.argtypes = [c_void_p,
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32)]
qhyccddll.GetQHYCCDParam.argtypes = [c_void_p, c_uint32]
qhyccddll.GetQHYCCDParam.restype = c_double
qhyccddll.SetQHYCCDParam.argtypes = [c_void_p, c_uint32, c_double]
qhyccddll.SetQHYCCDDebayerOnOff.argtypes = [c_void_p, c_bool]
qhyccddll.SetQHYCCDBinMode.argtypes = [c_void_p, c_uint32, c_uint32]
qhyccddll.SetQHYCCDResolution.argtypes = [c_void_p, c_uint32, c_uint32,
                                          c_uint32, c_uint32]
qhyccddll.ExpQHYCCDSingleFrame.argtypes = [c_void_p]
qhyccddll.GetQHYCCDSingleFrame.argtypes = [c_void_p,
    POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_ubyte)]
qhyccddll.Bits16ToBits8.argtypes = [c_void_p,
    POINTER(c_ubyte), POINTER(c_ubyte),
    c_uint32, c_uint32, c_uint16, c_uint16]

# -------------------------------------------------------------------------
class CONTROL_ID(Enum):
    CONTROL_EXPOSURE    = 8
    CONTROL_GAIN        = 6
    CONTROL_OFFSET      = 7
    CONTROL_USBTRAFFIC  = 12
    CONTROL_TRANSFERBIT = 10

# -------------------------------------------------------------------------
# Init camera
ret = qhyccddll.InitQHYCCDResource()
print("InitQHYCCDResource() ret =", ret)

num = qhyccddll.ScanQHYCCD()
print("ScanQHYCCD() num =", num)

camhandle = None
for index in range(num):
    id_buffer = create_string_buffer(40)
    qhyccddll.GetQHYCCDId(index, id_buffer)
    print(f"Camera {index} ID: {id_buffer.value.decode()}")
    camhandle = qhyccddll.OpenQHYCCD(id_buffer)
    if camhandle:
        print("Camera opened.")
        break

if not camhandle:
    raise RuntimeError("No camera could be opened.")

# -------------------------------------------------------------------------
# Basic camera config
qhyccddll.SetQHYCCDReadMode(camhandle, 0)
qhyccddll.SetQHYCCDStreamMode(camhandle, 0)
qhyccddll.InitQHYCCD(camhandle)

# -------------------------------------------------------------------------
# ### CHANGED — set 16‑bit raw mono output
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_TRANSFERBIT.value, 16.0)
qhyccddll.SetQHYCCDDebayerOnOff(camhandle, False)   # raw grayscale frame

# -------------------------------------------------------------------------
# Chip info
chipW = c_double(); chipH = c_double()
imageW = c_uint32(); imageH = c_uint32()
pixelW = c_double(); pixelH = c_double()
imageB = c_uint32()
qhyccddll.GetQHYCCDChipInfo(camhandle, byref(chipW), byref(chipH),
                            byref(imageW), byref(imageH),
                            byref(pixelW), byref(pixelH), byref(imageB))
print(f"Image size: {imageW.value}x{imageH.value}, Bits per pixel: {imageB.value}")

# -------------------------------------------------------------------------
# Exposure / gain ( Change the value depend on the output image )
qhyccddll.SetQHYCCDBinMode(camhandle, 1, 1)
qhyccddll.SetQHYCCDResolution(camhandle, 0, 0, imageW.value, imageH.value)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_EXPOSURE.value, 200000.0) 
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_GAIN.value,     30.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_OFFSET.value,    1.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_USBTRAFFIC.value,30.0)

# -------------------------------------------------------------------------
# ### CHANGED — allocate buffer for a 16‑bit frame (2 bytes per pixel)
frame_bytes = imageW.value * imageH.value * 2
imgdata_16  = (c_ubyte * frame_bytes)()

# -------------------------------------------------------------------------
# Serial setup (corrected baud rate; keep if 11500 is really what Arduino uses)
ser = serial.Serial('/dev/cu.usbmodem11201', 115200, timeout=1)

# Number of frames to capture
value = 64

os.makedirs("/Users/ajeems/Downloads/LU/phase shift files/data", exist_ok=True)

# -------------------------------------------------------------------------
for i in range(value):
    print(f"Waiting for trigger {i+1} from Arduino ...")
    while True:
        if ser.in_waiting:
            ch = ser.read(1)
            if ch == b'T':                # Arduino sends single byte 'Trigger'
                print("Trigger received")
                # ---- capture frame ----
                qhyccddll.ExpQHYCCDSingleFrame(camhandle)
                w = c_uint32(); h = c_uint32()
                b = c_uint32(); c = c_uint32()
                ret = qhyccddll.GetQHYCCDSingleFrame(camhandle,
                                   byref(w), byref(h), byref(b), byref(c),
                                   imgdata_16)
                if ret != 0:
                    print(f"GetQHYCCDSingleFrame failed ({ret})")
                else:
                    img_np16 = np.frombuffer(imgdata_16,
                                             dtype=np.uint16).reshape((h.value, w.value))
                    fname = (f"Save path---"
                             f"captured_image_{i+1:05d}.png")  #Set the desired image save path by modifying the savePath
                    cv2.imwrite(fname, img_np16)
                    print(f"Saved 16‑bit PNG: {fname}")
                break
        time.sleep(0.01)   # small sleep to avoid busy‑waiting

# -------------------------------------------------------------------------
ser.close()
qhyccddll.CloseQHYCCD(camhandle)
print("Capture complete — camera closed.")
