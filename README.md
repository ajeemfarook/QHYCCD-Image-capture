# QHYCCD‑Image‑Capture

A **minimal macOS-based image acquisition tool** to capture **phase-shift images** using a **QHY5III224C** camera, triggered by an **Arduino-controlled LED matrix**.

This project is primarily intended for research workflows involving synchronized image capture across LED patterns.

---

## ⚡ Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/yourusername/QHYCCD-Image-Capture.git
cd QHYCCD-Image-Capture

# 2. Install Python dependencies
python3 -m pip install opencv-python numpy pyserial

# 3. Install system dependencies (on macOS using Homebrew)
brew install opencv
# Download and install QHYCCD SDK manually from: https://www.qhyccd.com/download.html

# 4. Flash Arduino with LED sync code
# (115200 baud rate expected)
open arduino/PhaseShift_LED/PhaseShift_LED.ino

# 5. Edit library path and image save path in capture.py

# 6. Run the capture script
python3 capture.py
Captured 16-bit grayscale PNG images are saved in:

🖥️ Requirements
macOS 12+ (Monterey or newer)

QHYCCD SDK (v23.12 or later)

Python 3.10+

Libraries: opencv-python, numpy, pyserial

Arduino Uno/Nano with LED strip or 8x8 matrix (trigger signal on 'T')

🗂️ Repository Structure

QHYCCD-Image-Capture/
├── arduino/
│   └── PhaseShift_LED.ino    # Arduino sketch for LED trigger sync
├── capture.py                # Main image capture script
└── README.md                 # This file

🔧 Notes
Make sure to set the correct serial port in the serial.Serial() line of capture.py:

ser = serial.Serial('/dev/cu.usbmodem11201', 115200, timeout=1)
Use ls /dev/cu.* to find the correct device name for your Arduino.

The QHYCCD SDK must be downloaded from the official site and extracted. Then update the .dylib path in:

qhyccddll = cdll.LoadLibrary('your_sdk_path/libqhyccd.dylib')
