# Metalbot Firmware for Teensy 4.1

## Setup for uploading
  - Install Arduino
  - Install teensyduino 
    https://www.pjrc.com/teensy/td_download.html
  - Install micro_ros_arduino
    ```bash
    # Download zip archive of foxy branch
    https://github.com/micro-ROS/micro_ros_arduino/tree/foxy
    # Include it in your project using Sketch -> Include library -> Add .ZIP Library...

    export ARDUINO_PATH=[Your Arduino + Teensiduino path]
    cd $ARDUINO_PATH/hardware/teensy/avr/
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
    ```

  - Install arduino libraries
    BasicLinearAlgebra (Tom Stewart)
    Encoder (PaulStoffregen)
