#!/usr/bin/env python3
"""
Convert TIFF files to .mat format for FPM calibration
Ensures all images are loaded in RAW16 (uint16) format.
"""

import numpy as np
import scipy.io
from PIL import Image
import glob
import os

def load_tiff_stack(folder_path):
    """Load TIFF files into numpy array, validating RAW16 (uint16) format"""
    tiff_files = sorted(glob.glob(os.path.join(folder_path, "*.tif*")))
    
    if not tiff_files:
        raise ValueError(f"No TIFF files found in {folder_path}")
    
    # Load first image to get dimensions and validate
    first_img = Image.open(tiff_files[0])
    if first_img.mode != 'I;16':
        raise ValueError(f"First image '{tiff_files[0]}' is not in 16-bit RAW mode ('I;16'). Mode: {first_img.mode}. Re-capture as 16-bit TIFF.")
    first_arr = np.array(first_img)
    if first_arr.dtype != np.uint16:
        raise ValueError(f"First image '{tiff_files[0]}' is not uint16 dtype. Got: {first_arr.dtype}. Ensure raw 16-bit export.")
    
    # Handle RGB to grayscale (preserve 16-bit)
    if len(first_arr.shape) == 3:
        first_arr = np.mean(first_arr.astype(np.float64), axis=2).astype(np.uint16)  # Mean in float, back to uint16
    
    h, w = first_arr.shape
    num_images = len(tiff_files)
    
    # Create stack as uint16 to preserve RAW16
    stack = np.zeros((h, w, num_images), dtype=np.uint16)
    stack[:, :, 0] = first_arr
    
    print(f"Loaded {num_images} RAW16 images of size {h}x{w}")
    print(f"Image 0 range: {np.min(first_arr)} - {np.max(first_arr)} (uint16)")
    
    for i, tiff_file in enumerate(tiff_files[1:], 1):
        img = Image.open(tiff_file)
        if img.mode != 'I;16':
            raise ValueError(f"Image {i+1} '{tiff_file}' is not in 16-bit RAW mode ('I;16'). Mode: {img.mode}. Skipping invalid images? Fix capture settings.")
        arr = np.array(img)
        if arr.dtype != np.uint16:
            raise ValueError(f"Image {i+1} '{tiff_file}' is not uint16 dtype. Got: {arr.dtype}.")
        
        # Handle RGB to grayscale (preserve 16-bit)
        if len(arr.shape) == 3:
            arr = np.mean(arr.astype(np.float64), axis=2).astype(np.uint16)
        
        stack[:, :, i] = arr
        print(f"Image {i+1} range: {np.min(arr)} - {np.max(arr)} (uint16)")
    
    return stack

def create_led_array(num_leds, led_distance=74.0):
    """Generate LED array positions with specific ring diameters"""
    positions = []
    
    # DF ring: 58mm diameter = 29mm radius
    df_radius = 29.0 / led_distance  # Normalize by LED distance
    df_angles = np.linspace(0, 2*np.pi, 24, endpoint=False)
    for angle in df_angles:
        x = df_radius * np.cos(angle)
        y = df_radius * np.sin(angle)
        positions.append([x, y])
    
    # BF ring: 16mm diameter = 8mm radius
    bf_radius = 8.0 / led_distance  # Normalize by LED distance
    bf_angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    for angle in bf_angles:
        x = bf_radius * np.cos(angle)
        y = bf_radius * np.sin(angle)
        positions.append([x, y])
    
    return np.array(positions[:num_leds])

def create_mat_file(tiff_folder, output_file, params=None):
    """Convert TIFF stack to .mat file"""
    
    # Default parameters
    if params is None:
        params = {
            'na': 0.25,
            'mag': 10.0,
            'system_mag': 2.04,
            'pixel_size': 6.5,
            'wavelength': 0.514,
            'led_distance': 74.0
        }
    
    # Load images as RAW16 uint16
    images_uint16 = load_tiff_stack(tiff_folder)
    num_images = images_uint16.shape[2]
    
    # Convert to float64 only for .mat saving (normalize if needed; here preserves raw values)
    images = images_uint16.astype(np.float64)
    
    # Generate LED positions
    na_design = create_led_array(num_images, params['led_distance'])
    na_init = na_design + np.random.normal(0, 0.005, na_design.shape)
    
    # Calculate freqUV: expected spatial frequency of illumination (1/um)
    freqUV = na_design / params['wavelength']  # Mx2 array (x in col 1, y in col 2)
    
    # Classify brightfield/darkfield: first 24 are DF (1), last 8 are BF (0)
    dfi = np.ones((num_images, 1), dtype=bool)  # Start with all DF
    dfi[24:32] = 0  # Last 8 are BF
    
    # Create metadata structure
    metadata = {
        'objective': {
            'na': params['na'],
            'mag': params['mag'],
            'system_mag': params['system_mag']
        },
        'camera': {
            'pixel_size_um': params['pixel_size'],
            'is_color': False
        },
        'illumination': {
            'device_name': 'QHYCCD 526',  # Matches screenshot
            'wavelength_um': params['wavelength'],
            'z_distance_mm': float(params['led_distance'])  # Ensure float
        },
        'type': 'fpm',
        'file_header': 'Green Leds'
        ''
        ''
        ''
        ''
        '',  # Matches screenshot
        'source_list': {
            'na_design': na_design,
            'na_init': na_init,
            'freqUV': freqUV
        },
        'self_cal': {
            'na_cal': 0.2435,  # Matches screenshot
            'time_cal_s': 20.7287,  # Matches screenshot
            'DFI': dfi
        },
        'bk': np.zeros((2, num_images))
    }
    
    # Save to .mat file (data as float64, but raw values preserved)
    scipy.io.savemat(output_file, {
        'data': images,
        'metadata': metadata
    })
    
    print(f"\nCreated .mat file: {output_file}")
    print(f"Images: {images.shape} (loaded as RAW16 uint16, saved as float64)")
    print(f"LEDs: {num_images} ({np.sum(dfi==0)} BF, {np.sum(dfi==1)} DF)")

def main():
    # Default path for the 32 TIFF images
    tiff_folder = "/Users/ajeems/Downloads/LU/phase shift files/final ver/sample_9"
    output_file = "/Users/ajeems/Downloads/LU/phase shift files/final ver/green_2.mat"
    
    print(f"Using TIFF folder: {tiff_folder}")
    print(f"Output file: {output_file}")
    
    # System parameters
    print("\nSystem parameters (press Enter for defaults):")
    na = input("Objective NA [0.25]: ") or "0.25"
    mag = input("Magnification [10.0]: ") or "10.0"
    wavelength = input("Wavelength μm [0.514]: ") or "0.514"
    pixel_size = input("Pixel size μm [6.5]: ") or "6.5"
    
    params = {
        'na': float(na),
        'mag': float(mag),
        'system_mag': 2.04,
        'pixel_size': float(pixel_size),
        'wavelength': float(wavelength),
        'led_distance': 74.0
    }
    
    create_mat_file(tiff_folder, output_file, params)

if __name__ == "__main__":
    main()