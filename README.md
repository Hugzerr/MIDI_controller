# Midi controller

Following this [tuto](https://hackaday.io/project/186417-usb-midi-driver-for-stm32cubemx-stm32cubeide)

Generated a project from STM32CubeMX with the USB_device middleware set as HID. (using Nucleo144-F767ZI for secondary usb port)

# TODO

- Receive signals
- Connect sensors
  - Switch
  - Push button
  - Rotary encoder
  - Optical encoder

# Done

- Sensors
  - Rotary potentiometer (CC 55)
  - Fader (CC 56) - same code as rotary pot
- Add MIDI support
  - MIDI middleware based on modded HID
- Send midi messages
