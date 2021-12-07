# DACDriver

Implements an USB audio device (2 channels, 48kHz) and plays the data through 12-bit DAC1 (left audio channel) and DAC2 (right audio channel).

When there is no data from USB in 500ms the devices starts to play 50Hz sin and cos wave to DAC channels. Feature can be disabled with a flag.

Developed on 
 * STM32F407-DISCOVERY1 board.
 * System Workbench for STM32

Intended for driving medium frequency DAC signal from a PC over USB.

Build with CMake
================
Get CMake from https://cmake.org/ . 
Install your preferred build system. In this example ninja is used (https://ninja-build.org/).
Standalone compiler suite without IDE can be downloaded from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm.

1. Make sure that correct arm compiler (arm-none-eabi-gcc) is in path.
2. Create build directory
	mkdir build
	cd build
3. Generate build files using provided toolchain file
	cmake .. -G Ninja -DCMAKE_TOOLCHAIN_FILE=..\toolchain-gnu-stm.cmake
   or Debug build configuration
	cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=..\toolchain-gnu-stm.cmake
4. Build the firmware
	cmake --build .

Build configuration can be selected also with cmake-gui
	cmake-gui .


Build with System Workbench for STM32
=====================================
The latest version of the IDE can be downloaded from http://www.ac6-tools.com/downloads/SW4STM32/install_sw4stm32_win_64bits-latest.exe .

Minimum version of Workbench that should be used for compiling is v4.6.3





