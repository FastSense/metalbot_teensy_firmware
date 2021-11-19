# Metalbot Firmware for Teensy 4.1

## Setup for uploading

### Install Arduino IDE
https://www.arduino.cc/en/software

### Install Teensyduino
https://www.pjrc.com/teensy/td_download.html

### Install libraries
#### Micro-ROS:
  Download zip archive of foxy branch: https://github.com/micro-ROS/micro_ros_arduino/tree/foxy
  Include it in your project using Sketch -> Include library -> Add .ZIP Library...
  And Setup platform:
      ```bash
      export ARDUINO_PATH=[Your Arduino + Teensyduino path]
      cd $ARDUINO_PATH/hardware/teensy/avr/
      curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
      ```
#### Arduino libraries:
 Tools -> Manage Libraries...
  - BasicLinearAlgebra (Tom Stewart)
  - Encoder (Paul Stoffregen)

### Get HEX-file
  - Arduino IDE: Compile (c-R)
  - Teensyduino: File -> Open HEX File

### Upload firmware
  - Use scp to transfer firmware
  - Use tycmd to upload
  ```bash
  sudo tycmd upload <filename.hex>
  (https://koromix.dev/tytools#tycmd_upload)
  ```
