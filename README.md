# Midi controller

Following this [tuto](https://hackaday.io/project/186417-usb-midi-driver-for-stm32cubemx-stm32cubeide)

Generated a project from STM32CubeMX with the USB_device middleware set as HID. (using Nucleo144-F767ZI for secondary usb port)

# TODO

- Clean code
  - Remove or fix MIDI middleware based on working modded HID
  - Remove redondant includes (in main and cmakeList)
- Send and receive signals
- Connect sensors

# Done

- Add MIDI support
