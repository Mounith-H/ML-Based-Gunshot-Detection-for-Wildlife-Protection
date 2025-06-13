@echo off
setlocal enabledelayedexpansion

echo Installing libraries for Gunshot Detection Project...

:: Create lib directory if it doesn't exist
if not exist lib mkdir lib

:: Function to clone or update a git repository
:clone_or_update
set repo_url=%~1
set folder_name=%~2

if exist lib\%folder_name%\ (
    echo Updating %folder_name%...
    cd lib\%folder_name%
    git pull
    cd ..\..
) else (
    echo Cloning %folder_name%...
    git clone %repo_url% lib\%folder_name%
)
goto :eof

:: Function to ensure minmea compiler definition exists
:ensure_minmea_config
echo Checking minmea compiler definitions...
findstr /C:"target_compile_definitions(minmea PRIVATE" CMakeLists.txt >nul
if errorlevel 1 (
    echo Adding minmea compiler definitions to CMakeLists.txt...
    powershell -Command "& {
        $content = Get-Content CMakeLists.txt -Raw
        $pattern = '(?ms)(target_include_directories\(minmea PUBLIC.*?\))'
        $replacement = '$1

# Add compiler definitions for minmea
target_compile_definitions(minmea PRIVATE
    timegm=mktime  # Use mktime instead of timegm
)'
        $content -replace $pattern, $replacement | Set-Content CMakeLists.txt
    }"
) else (
    echo Minmea compiler definitions already exist in CMakeLists.txt
)
goto :eof

:: Install minmea GPS parser library
call :clone_or_update "https://github.com/kosma/minmea.git" "minmea"

:: Install NRF24L01 library
call :clone_or_update "https://github.com/MikulasP/Pico_NRF24L01" "Pico_NRF24L01"

:: Install pico-tflmicro library
call :clone_or_update "https://github.com/raspberrypi/pico-tflmicro.git" "pico-tflmicro"

:: Install TensorFlow Lite Micro
if not exist lib\tensorflow-lite-micro\ (
    echo Setting up TensorFlow Lite Micro...
    git clone https://github.com/tensorflow/tflite-micro.git lib\tensorflow-lite-micro
    cd lib\tensorflow-lite-micro
    make -f tensorflow\lite\micro\tools\make\Makefile third_party_downloads
    cd ..\..
)

:: Ensure minmea compiler definitions are present
call :ensure_minmea_config

:: Set up build environment
echo Setting up build environment...
if not exist build mkdir build
cd build

:: Configure CMake
echo Configuring CMake...
cmake ..

echo Installation complete!
echo You can now build the project using: 'cd build ^& cmake --build .'

pause