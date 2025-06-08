# Gunshot Detection System - Raspberry Pi Pico Implementation

This directory contains the Raspberry Pi Pico implementation of the ML-based gunshot detection system. The system uses PIO state machines for efficient peripheral control and implements real-time audio processing with machine learning capabilities.

## Directory Structure
```
├── CMakeLists.txt          # Main CMake configuration
├── gps_handler.*          # GPS module interface
├── gunshot_model.*        # TensorFlow Lite model interface
├── inmp441.pio           # PIO program for INMP441 microphone
├── model_handler.*        # ML model management
├── power_control.pio     # PIO program for IP5306 power control
├── Raspberry_pi_pico.cpp # Main application
└── time_utils.h          # Time management utilities
```

## Prerequisites
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- CMake (version 3.12 or higher)
- C/C++ compiler (GCC for Linux, MSVC for Windows)
- Git
- Make (for Linux)

## Installation

### 1. Set Up Environment Variables
```bash
# For Linux
export PICO_SDK_PATH=/path/to/pico-sdk

# For Windows (PowerShell)
$env:PICO_SDK_PATH="C:\path\to\pico-sdk"
```

### 2. Install Required Libraries
For Linux:
```bash
chmod +x setup_linux.sh
./setup_linux.sh
```

For Windows:
```cmd
setup_windows.bat
```

This will install:
- minmea (GPS NMEA parser)
- NRF24L01 radio library
- TensorFlow Lite Micro

### 3. Build the Project
```bash
mkdir build
cd build
cmake ..
make
```

## Hardware Configuration

### Required Components
- Raspberry Pi Pico
- INMP441 MEMS Microphone
- NEO-6M GPS Module
- NRF24L01 Radio Module
- IP5306 Power Management IC

### Pin Connections

#### INMP441 Microphone
- VDD → 3.3V (Pin 36)
- GND → GND (Pin 38)
- SD → GP20 (Pin 26)
- SCK → GP18 (Pin 24)
- WS → GP19 (Pin 25)
- L/R → GND

#### GPS Module (NEO-6M)
- VCC → 3.3V (Pin 36)
- GND → GND (Pin 38)
- TX → GP9 (Pin 11)
- RX → GP8 (Pin 12)

#### NRF24L01
- VCC → 3.3V (Pin 36)
- GND → GND (Pin 38)
- SCK → GP2 (Pin 4)
- MOSI → GP3 (Pin 5)
- MISO → GP4 (Pin 6)
- CSN → GP5 (Pin 7)
- CE → GP6 (Pin 9)
- IRQ → GP7 (Pin 10)

#### Power Control (IP5306)
- KEY → GP15 (Pin 20)

## Key Features

### 1. Power Management
- PIO-based PWM control for IP5306
- Continuous KEY pin pulsing
- 16-bit resolution for smooth control
- Automatic power maintenance

### 2. Audio Processing
- Real-time audio sampling via PIO
- DMA-based data transfer
- Configurable sampling rate
- Efficient ring buffer implementation

### 3. GPS Integration
- NMEA message parsing
- Position and time synchronization
- Configurable update rate
- Low-power operation support

### 4. Wireless Communication
- Packet fragmentation
- Automatic retry mechanism
- Mesh network capability
- Error detection and correction

## Development Notes

### Adding New Features
1. Create new source files in the root directory
2. Update CMakeLists.txt to include new sources
3. Run CMake configuration again

### Debugging
- Serial output available on UART0 (USB)
- Debug prints can be enabled in CMakeLists.txt
- LED indicators for system status

### Power Optimization
- Core 1 can be disabled when not needed
- PIO programs are power-efficient
- GPS can be put into low-power mode
- Radio duty cycling implemented

## Common Issues and Solutions

### Build Issues
1. **CMake Error**: Ensure PICO_SDK_PATH is set correctly
2. **Compiler Error**: Check GCC/G++ installation
3. **Library Error**: Run setup_linux or setup_windows script again

### Hardware Issues
1. **No Power**: Check IP5306 connections and KEY signal
2. **No GPS Fix**: Ensure clear view of sky
3. **Radio Issues**: Check antenna and power supply

## Tasks
Available VS Code tasks:
- `Compile Project`: Builds the project
- `Run Project`: Loads program via picotool
- `Flash`: Programs via OpenOCD/CMSIS-DAP

## Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License
This project is licensed under the MIT License - see the LICENSE file for details
