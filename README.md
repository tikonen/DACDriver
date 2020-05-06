# DACDriver

Implements an USB audio device (2 channels, 48kHz) and plays the data through 12-bit DAC1 (left audio channel) and DAC2 (right audio channel).

When there is no data from USB in 500ms the devices starts to play 50Hz sin and cos wave to DAC channels. Feature can be disabled with a flag.

Developed on 
 * STM32F407-DISCOVERY1 board.
 * System Workbench for STM32

Intended for driving medium frequency DAC signal from a PC over USB.
