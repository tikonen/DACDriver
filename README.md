# DACDriver

Implements an USB audio device (2 channels, 48kHz) that plays the data through boards 12-bit DAC1 (left audio channel) and DAC2 (right audio channel). Intended for driving a medium frequency analog signal from a PC over USB.

When there is no data from USB in 500ms the devices starts to play 50Hz sin and cos wave to DAC channels. This project was built as part of project to drive vector graphics to Oscilloscope as sound, the feature was added as a "screensaver" for the CRT Oscilloscopes. For details see https://github.com/tikonen/AudioRender.
Feature can be disabled by the connecting board pin PA0 to 3v3.

Developed on
 * STM32F407-DISCOVERY1 board.
 * System Workbench for STM32


# Build with CMake

### Prerequisities

Get CMake from https://cmake.org/.

Install your preferred build system. In this example ninja is used (https://ninja-build.org/).

Standalone compiler suite can be found at https://developer.arm.com/open-source/gnu-toolchain/gnu-rm.

### Generate build files.

Following examples use the [Git-Bash environment](https://git-scm.com/download/win).

1. Make sure that correct arm compiler (arm-none-eabi-gcc) is in path.
    ```
    /c/projects/DACDriver $ arm-none-eabi-gcc --version
    arm-none-eabi-gcc.exe (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
    Copyright (C) 2019 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    ```


2. Create build directory
    ```
    $ mkdir build
	$ cd build
    ```
3. Generate build files for your build system using provided toolchain file
    ```
    $ cmake .. -G Ninja -DCMAKE_TOOLCHAIN_FILE=..\toolchain-gnu stm.cmake
    ```

   or Debug build configuration
	```
    $ cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug DCMAKE_TOOLCHAIN_FILE=..\toolchain-gnu-stm.cmake
    ```
4. Build the firmware
    ```
    $ cmake --build .
    ```

Build configuration can also be done  with cmake-gui.
```
$ mkdir build
$ cmake-gui -S . -B build
```

# Build with System Workbench for STM32

The latest version of the IDE can be downloaded from http://www.ac6-tools.com/downloads/SW4STM32/install_sw4stm32_win_64bits-latest.exe .

Minimum version of Workbench that should be used for compiling is v4.6.3
