# Interface for Daimon Tactile Sensor

This directory contains Python scripts for interfacing with DM-Tac tactile sensors. The scripts provide real-time visualization and data processing capabilities for single and dual sensor setups.

## Prerequisites

- Python 3.8/3.9/3.10
- CUDA toolkit 12.x (required for GPU acceleration)
- Physical DM-Tac tactile sensor(s) connected via USB

## Installation

1. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Install the dmrobotics package:
   ```bash
   pip install .
   ```

## Available Scripts

### 1. `main.py` - Single Sensor Interface
Basic interface for a single DM-Tac tactile sensor.

**Features:**
- Real-time visualization of raw image, depth map, deformation, and shear data
- FPS monitoring
- Sensor reset capability

**Usage:**
```bash
python main.py
```

**Controls:**
- `q`: Quit the application
- `r`: Reset the sensor

**Output Windows:**
- `img`: Raw sensor image
- `depth`: Depth map visualization (heat map)
- `deformation`: 2D deformation vectors
- `shear`: Shear force visualization

### 2. `dual_main.py` - Dual Sensor Interface
Advanced interface for simultaneously controlling two DM-Tac tactile sensors.

**Features:**
- Simultaneous operation of two sensors (Device IDs 0 and 1)
- Combined visualization in a 2x2 grid layout
- Graceful error handling for single sensor mode
- Enhanced display with sensor labels
- Keyboard interrupt handling (Ctrl+C)

**Usage:**
```bash
python dual_main.py
```

**Controls:**
- `q`: Quit the application
- `r`: Reset both sensors
- `Ctrl+C`: Graceful shutdown

**Output:**
- Combined visualization window showing deformation and shear data from both sensors
- Automatic fallback to single sensor mode if second sensor is unavailable

**Sensor Configuration:**
- Sensor 1: Device ID 0 (Serial: 21CAF26C)
- Sensor 2: Device ID 1 (Serial: 1EBE4E51)

### 3. `test_feedback.py` - Feedback Testing Interface
Specialized script for testing sensor feedback with baseline calibration.

**Features:**
- Baseline depth calibration on startup
- Real-time feedback metrics (max intensity and pixel sum)
- Baseline-adjusted depth measurements
- Focused on depth visualization for testing purposes

**Usage:**
```bash
python test_feedback.py
```

**Controls:**
- `q`: Quit the application
- `r`: Reset sensor and update baseline

**Output:**
- Real-time console output showing max intensity and pixel sum values
- Depth visualization window
- Baseline-adjusted measurements for consistent feedback testing

## Hardware Setup

1. Connect the DM-Tac sensor(s) via USB
2. Ensure proper power supply (if required)
3. Check device recognition in system

## Troubleshooting

- **CUDA Issues**: If you encounter CUDA-related errors, ensure you have CUDA toolkit 12.x installed
- **Sensor Not Detected**: Check USB connections and device drivers
- **Performance Issues**: Ensure GPU drivers are up to date for optimal performance
- **Dual Sensor Mode**: If only one sensor is detected, the system will automatically fall back to single sensor mode

## Dependencies

The following packages are required (see `requirements.txt`):
- numpy==1.24.4
- opencv-python==4.10.0.84
- opencv-contrib-python==4.10.0.84
- scipy==1.10.1
- setuptools==45.2.0
- cupy-cuda12x==12.4.99

## Documentation

For detailed product specifications, refer to `DM-Tac W产品说明书 V1.0 20250417.pdf` in this directory.
