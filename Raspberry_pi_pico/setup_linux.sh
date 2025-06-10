#!/bin/bash

echo "Installing libraries for Gunshot Detection Project..."

# Create lib directory if it doesn't exist
mkdir -p lib

# Function to clone or update a git repository
clone_or_update() {
    local repo_url=$1
    local folder_name=$2
    
    if [ -d "lib/$folder_name" ]; then
        echo "Updating $folder_name..."
        cd "lib/$folder_name"
        git pull
        cd ../..
    else
        echo "Cloning $folder_name..."
        git clone "$repo_url" "lib/$folder_name"
    fi
}

# Function to ensure minmea compiler definition exists in CMakeLists.txt
ensure_minmea_config() {
    local cmake_file="CMakeLists.txt"
    if ! grep -q "target_compile_definitions(minmea PRIVATE" "$cmake_file"; then
        # Find the minmea target_include_directories block and add our definitions after it
        awk '/target_include_directories\(minmea PUBLIC/{p=NR+3}(NR<=p){print}/^)/{if(NR==p+1){print "\n# Add compiler definitions for minmea\ntarget_compile_definitions(minmea PRIVATE\n    timegm=mktime  # Use mktime instead of timegm\n)"}}' "$cmake_file" > "${cmake_file}.tmp"
        mv "${cmake_file}.tmp" "$cmake_file"
        echo "Added minmea compiler definitions to CMakeLists.txt"
    else
        echo "Minmea compiler definitions already exist in CMakeLists.txt"
    fi
}

# Install minmea GPS parser library
clone_or_update "https://github.com/kosma/minmea.git" "minmea"

# Install NRF24L01 library
clone_or_update "https://github.com/MikulasP/Pico_NRF24L01" "Pico_NRF24L01"

# Install pico-tflmicro library
clone_or_update "https://github.com/raspberrypi/pico-tflmicro.git" "pico-tflmicro"

# Install TensorFlow Lite Micro
if [ ! -d "lib/tensorflow-lite-micro" ]; then
    echo "Setting up TensorFlow Lite Micro..."
    git clone https://github.com/tensorflow/tflite-micro.git lib/tensorflow-lite-micro
    cd lib/tensorflow-lite-micro
    make -f tensorflow/lite/micro/tools/make/Makefile third_party_downloads
    cd ../..
fi

# Ensure minmea compiler definitions are present
ensure_minmea_config

# Set up build environment
echo "Setting up build environment..."
mkdir -p build
cd build

# Configure CMake
echo "Configuring CMake..."
cmake ..

echo "Installation complete!"
echo "You can now build the project using: 'cd build && make'"
