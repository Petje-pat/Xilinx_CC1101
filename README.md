# Xilinx_CC1101
This library is based is the arduino library from https://github.com/LSatan/SmartRC-CC1101-Driver-Lib. Full documentation and wire diagrams can be found there.
The library depends on the XSpiPs.h and Gpio.h library to communicate over SPI and receive state information over gpio pins.

## features
 - [x] Controlling the module and sending and receiving data.
 - [X] Examples work by polling
 - [ ] Interrupt based.
 - [X] Should work with and without System Device Tree (SDT) as introduced in Vitis 2023.2
 
