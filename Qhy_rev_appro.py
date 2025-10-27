import cv2
import numpy as np
from ctypes import (create_string_buffer, cdll, byref, c_void_p, c_uint32,
                    c_char_p, c_double, c_bool, c_ubyte, POINTER)
from enum import Enum
import warnings
import serial
import time
from datetime import datetime
import os


warnings.filterwarnings("ignore", category=DeprecationWarning)

# Load SDK
qhyccddll = cdll.LoadLibrary(
    '/Users/ajeems/Downloads/LU/phase shift files/dev files/sdk_mac_arm_25.06.16/usr/local/lib/libqhyccd.dylib'
)

# Function prototypes
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
qhyccddll.GetQHYCCDChipInfo.argtypes = [
    c_void_p,
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
qhyccddll.GetQHYCCDSingleFrame.argtypes = [
    c_void_p,
    POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_ubyte)
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

# Disable debayering and use 16-bit output
qhyccddll.SetQHYCCDDebayerOnOff(camhandle, False)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_TRANSFERBIT.value, 16.0)

# Get chip info
chipW = c_double()
chipH = c_double()
imageW = c_uint32()
imageH = c_uint32()
pixelW = c_double()
pixelH = c_double()
imageB = c_uint32()
qhyccddll.GetQHYCCDChipInfo(
    camhandle, byref(chipW), byref(chipH), byref(imageW), byref(imageH),
    byref(pixelW), byref(pixelH), byref(imageB)
)
print(f"Image size: {imageW.value}x{imageH.value}, Bits per pixel: {imageB.value}")

# Set binning and resolution
qhyccddll.SetQHYCCDBinMode(camhandle, 1, 1)
qhyccddll.SetQHYCCDResolution(camhandle, 0, 0, imageW.value, imageH.value)

# Set exposure and gain
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_GAIN.value, 30.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_USBTRAFFIC.value, 15.0)

# Allocate buffer for 16-bit Bayer raw image
frame_bytes = imageW.value * imageH.value * 2  # 2 bytes per pixel
imgdata_raw16 = (c_ubyte * frame_bytes)()

# --- Setup serial connection for Arduino (use first successful port) ---
ports = [
    '/dev/tty.usbmodem1301',  # Include tty for reliability
    '/dev/cu.usbmodem1301',  
    '/dev/cu.usbmodem11301', 
    '/dev/cu.usbmodem1401',    # Your actual port first
    '/dev/cu.usbmodem1201',
    '/dev/cu.usbmodem11201'
]

serial_connections = []
ser = None
for port in ports:
    try:
        ser_temp = serial.Serial(port, 115200, timeout=1)
        serial_connections.append(ser_temp)
        ser = ser_temp
        print(f"Connected to Arduino on {port}")
        break  # Use the first successful connection
    except serial.SerialException:
        print(f"Failed to connect to {port}")

if ser is None:
    raise RuntimeError("No Arduino serial port could be opened.")

time.sleep(2)  # Allow Arduino reset

BIG_RING_SIZE = 24
value = 24  # number of captures
LED_HOLD_MS = 200  # LED hold duration (ms)
SUB_EXPOSURE_US = 50000  # 50ms per sub-exposure (total 4 = 200ms)
NUM_SUB_CAPTURES = 4  # Multiple short exposures during hold
CAPTURE_SETTLE_DELAY = 0.05  # 50ms delay after LED ON for peak

# --- Base folder where captured runs will be saved ---
base_dir = "/Users/ajeems/Downloads/LU/phase shift files/final ver"
folder_prefix = "sample"

# --- Create a new run folder automatically ---
run_index = 1
while os.path.exists(os.path.join(base_dir, f"{folder_prefix}_{run_index}")):
    run_index += 1

save_dir = os.path.join(base_dir, f"{folder_prefix}_{run_index}")
os.makedirs(save_dir)
print(f"Saving results in: {save_dir}")

try:
    # Set idle brightness to 10 for all red idle mode
    ser.write(b'IDLE_BRIGHT:10\n')
    ser.flush()
    print("Idle brightness set to 10")

    # Set sub-exposure once (short for multiple captures)
    qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_EXPOSURE.value, SUB_EXPOSURE_US)

    # --- PC-Master Capture Loop with LED Sync + Multiple Sub-Exposures ---
    for i in range(value):
        print(f"Starting PC-guided capture {i+1}/{value}...")

        # Step 1: Signal Arduino LED ON (cycle LED index, hold for total duration)
        led_index = i % BIG_RING_SIZE  # Cycle through 24 LEDs
        led_start_time = time.time()
        ser.write(f'LIGHT:{led_index}:{LED_HOLD_MS}\n'.encode())
        ser.flush()
        print(f"LED {led_index} ON for {LED_HOLD_MS}ms sent at {led_start_time:.6f}")

        # Step 2: Brief settle time for LED to reach max intensity + camera sync
        time.sleep(0.001 + CAPTURE_SETTLE_DELAY)  # 1ms LED + 50ms for peak

        # Step 3: Multiple sub-exposures during hold, pick brightest red channel
        capture_start_time = time.time()
        sub_frames = []
        sub_brightnesses = []
        for j in range(NUM_SUB_CAPTURES):
            print(f"  Sub-exposure {j+1}/{NUM_SUB_CAPTURES}...")
            qhyccddll.ExpQHYCCDSingleFrame(camhandle)

            w = c_uint32()
            h = c_uint32()
            b = c_uint32()
            c = c_uint32()

            try:
                ret = qhyccddll.GetQHYCCDSingleFrame(
                    camhandle, byref(w), byref(h), byref(b), byref(c), imgdata_raw16
                )
            except Exception as e:
                print(f"  Sub-camera exception: {e}")
                continue

            if ret != 0:
                print(f"  Failed sub-frame {j+1}, error: {ret}")
                continue

            # Convert to NumPy and demosaic
            img_np_1d = np.ctypeslib.as_array(imgdata_raw16)
            img_np = img_np_1d.view(np.uint16).reshape((h.value, w.value))
            rgb_img = cv2.cvtColor(img_np, cv2.COLOR_BayerGB2RGB)
            red_channel = rgb_img[:, :, 2].astype(np.float32)

            # Compute mean red channel brightness
            mean_bright = np.mean(red_channel)
            sub_frames.append(red_channel)
            sub_brightnesses.append(mean_bright)
            print(f"  Sub-brightness {j+1}: {mean_bright:.1f}")

        # Pick and save the brightest sub-frame (most likely during lit LED peak)
        if sub_frames:
            brightest_idx = np.argmax(sub_brightnesses)
            best_red_channel = sub_frames[brightest_idx]
            best_bright = sub_brightnesses[brightest_idx]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_filename = os.path.join(save_dir, f"captured_red_{i+1}_{timestamp}_bright{best_bright:.0f}.png")
            cv2.imwrite(img_filename, best_red_channel)
            print(f"Best sub-image saved: {img_filename} (brightness {best_bright:.1f})")
        else:
            print(f"No sub-frames for capture {i+1}")

        capture_end_time = time.time()
        print(f"Capture complete at {capture_end_time:.6f}")

        # --- Timing (jitter-focused) ---
        duration = capture_end_time - capture_start_time
        led_to_capture_latency = capture_start_time - led_start_time
        print(
            f"Timing -> Sub-Capture Duration: {duration:.6f}s, LED-ON to Sub-Capture Latency: {led_to_capture_latency:.6f}s"
        )

        # Step 5: Wait remaining hold time if needed (total ~ LED hold)
        remaining_hold = LED_HOLD_MS / 1000.0 - (capture_end_time - led_start_time)
        if remaining_hold > 0:
            time.sleep(remaining_hold)

        # No explicit OFF—Arduino auto-clears after hold duration

        # Optional: Brief pause between captures
        time.sleep(0.05)

    print("PC-guided sequence complete—all captures with LED at max during exposure!")

except Exception as e:
    print(f"Error during sequence: {e}")
    if ser:
        ser.write(b'OFF\n')  # Ensure OFF on error
        ser.flush()

finally:
    # --- Cleanup ---
    qhyccddll.CloseQHYCCD(camhandle)
    for s in serial_connections:
        s.close()
    print("Camera and serial closed. Images saved in folder: " + save_dir)
