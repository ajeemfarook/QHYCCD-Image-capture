import cv2
import numpy as np
from ctypes import create_string_buffer
from ctypes import cdll, byref, c_void_p, c_uint32, c_char_p, c_double, c_bool, c_ubyte, POINTER, c_uint16
from enum import Enum
import warnings
import serial
import time
import os

warnings.filterwarnings("ignore", category=DeprecationWarning)

# Load SDK
qhyccddll = cdll.LoadLibrary('/Users/ajeems/Downloads/LU/phase shift files/dev files/sdk_mac_arm_25.06.16/usr/local/lib/libqhyccd.dylib')

# Define required function prototypes
qhyccddll.GetQHYCCDId.argtypes = [c_uint32, c_char_p]
qhyccddll.OpenQHYCCD.argtypes = [c_char_p]
qhyccddll.OpenQHYCCD.restype = c_void_p
qhyccddll.CloseQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDNumberOfReadModes.argtypes = [c_void_p, POINTER(c_uint32)]
qhyccddll.GetQHYCCDReadModeName.argtypes = [c_void_p, c_uint32, c_char_p]
qhyccddll.GetQHYCCDReadModeResolution.argtypes = [c_void_p, c_uint32, POINTER(c_uint32), POINTER(c_uint32)]
qhyccddll.SetQHYCCDReadMode.argtypes = [c_void_p, c_uint32]
qhyccddll.SetQHYCCDStreamMode.argtypes = [c_void_p, c_uint32]
qhyccddll.InitQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDChipInfo.argtypes = [c_void_p,
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32)
]
qhyccddll.GetQHYCCDParam.argtypes = [c_void_p, c_uint32]
qhyccddll.GetQHYCCDParam.restype = c_double
qhyccddll.SetQHYCCDParam.argtypes = [c_void_p, c_uint32, c_double]
qhyccddll.SetQHYCCDDebayerOnOff.argtypes = [c_void_p, c_bool]
qhyccddll.SetQHYCCDBinMode.argtypes = [c_void_p, c_uint32, c_uint32]
qhyccddll.SetQHYCCDResolution.argtypes = [c_void_p, c_uint32, c_uint32, c_uint32, c_uint32]
qhyccddll.ExpQHYCCDSingleFrame.argtypes = [c_void_p]
qhyccddll.GetQHYCCDSingleFrame.argtypes = [c_void_p,
    POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_ubyte)
]
qhyccddll.Bits16ToBits8.argtypes = [c_void_p,
    POINTER(c_ubyte), POINTER(c_ubyte),
    c_uint32, c_uint32, c_uint16, c_uint16
]

# Enum for camera controls
class CONTROL_ID(Enum):
    CONTROL_EXPOSURE = 8
    CONTROL_GAIN = 6
    CONTROL_OFFSET = 7
    CONTROL_USBTRAFFIC = 12
    CONTROL_TRANSFERBIT = 10

# Initialize SDK
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

# Basic config
qhyccddll.SetQHYCCDReadMode(camhandle, 0)
qhyccddll.SetQHYCCDStreamMode(camhandle, 0)
qhyccddll.InitQHYCCD(camhandle)

# Set parameters
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_TRANSFERBIT.value, 8.0)  # 8-bit output for RGB
qhyccddll.SetQHYCCDDebayerOnOff(camhandle, True)  # Enable debayer (color)

# Get chip info
chipW = c_double()
chipH = c_double()
imageW = c_uint32()
imageH = c_uint32()
pixelW = c_double()
pixelH = c_double()
imageB = c_uint32()
qhyccddll.GetQHYCCDChipInfo(camhandle, byref(chipW), byref(chipH), byref(imageW), byref(imageH), byref(pixelW), byref(pixelH), byref(imageB))
print(f"Image size: {imageW.value}x{imageH.value}, Bits per pixel: {imageB.value}")

# Set bin and ROI
qhyccddll.SetQHYCCDBinMode(camhandle, 1, 1)
qhyccddll.SetQHYCCDResolution(camhandle, 0, 0, imageW.value, imageH.value)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_EXPOSURE.value, 200000.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_GAIN.value, 30.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_OFFSET.value, 1.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_USBTRAFFIC.value, 30.0)

# Allocate buffer for 8-bit 3-channel image
frame_bytes = imageW.value * imageH.value * 3  # 3 channels (RGB)
imgdata_rgb = (c_ubyte * frame_bytes)()

# Setup serial port
ser = serial.Serial('/dev/cu.usbmodem11201',11500, timeout=1)

#different number of frames
value = 64        # <‑‑ edit this number

# Wait for triggers from Arduino
for i in range(value):
    print(f"Waiting for trigger {i+1} from Arduino...")

    while True:
        if ser.in_waiting > 0:
            data = ser.read(1)
            if data == b'Trigger':  # Arduino sends 'Trigger' as trigger
                print("Trigger received!")

                # Capture frame
                qhyccddll.ExpQHYCCDSingleFrame(camhandle)
                w = c_uint32()
                h = c_uint32()
                b = c_uint32()
                c = c_uint32()
                ret = qhyccddll.GetQHYCCDSingleFrame(camhandle, byref(w), byref(h), byref(b), byref(c), imgdata_rgb)
                if ret != 0:
                    print(f"Failed to get frame {i}, error code: {ret}")
                else:
                    print(f"Captured frame {i}: {w.value}x{h.value}, bpp={b.value}, channels={c.value}")

                    img_np = np.ctypeslib.as_array(imgdata_rgb).reshape((h.value, w.value, 3))
                    os.makedirs("/Users/ajeems/Downloads/LU/phase shift files/data", exist_ok=True)
                    filename = f"/Users/ajeems/Downloads/LU/phase shift files/data/captured_image_{i+1}.png"
                    cv2.imwrite(filename, img_np)
                    print(f"Image saved: {filename}")
                break
        time.sleep(0.01)  # small delay to avoid busy wait

# Cleanup
qhyccddll.CloseQHYCCD(camhandle)