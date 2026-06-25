# Midi controller

First prototype of my MIDI DJ controller.

Case fully 3D printed, with basic components and STM32.

Following this [tuto](https://hackaday.io/project/186417-usb-midi-driver-for-stm32cubemx-stm32cubeide)

Generated a project from STM32CubeMX with the USB_device middleware set as HID. (using Nucleo144-F767ZI for secondary usb port)

# TODO

## MVP

- Connect sensors
  - Push button
  - Rotary encoder
  - Optical encoder

## After

- Receive signals

# Done

- Sensors
  - Rotary potentiometer (CC 55)
  - Fader (CC 56) - same code as rotary pot
  - Switch (CC 57)
- Add MIDI support
  - MIDI middleware based on modded HID
- Send midi messages
