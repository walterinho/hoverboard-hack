# Hoverboard - Speed control Firmware

This repo contains open source firmware for generic Hoverboard Mainboards.

This project uses UART DMA to receive two floats on USART2 with 115200 baud, and uses these floats to control the motor current of each motor. It also counts the hall ticks and sends back the motor positions to use them for odometry data, for example combined with ROS.


## Setup software to compile and upload STM32 code in ubuntu

1. Install GNU tool chain using command:

  $ sudo apt-get install gcc-arm-none-eabi

also install GDBServer used to debug code:

  $ sudo apt install gdb-arm-none-eabi

2. Download the STM32 Firmware Library from the link below, and put it in an  appropiate place.

http://www.st.com/en/embedded-software/stsw-stm32054.html

3. Download ST-LINK software from

https://github.com/texane/stlink

4. Execute the following commands inside the root of the source directory:

$ make release
$ make debug (for debug purposes)
$ cd build/Release
$ make install
$ ldconfig (updates the dynamic library cache, fixes shared library bug)

5. Permissions with udev, The rules are located in the etc/udev/rules.d directory. You will need to copy it to /etc/udev/rules.d and run:

$ udevadm control –reload-rules
$ udevadm trigger

6. Finally compile the code inside the project folder using the command make to get the .hex file

## Hardware explanation

![Alt text](schema.jpg?raw=true "Mainboard diagram")

The original Hardware supports two 4-pin cables that originally were connected to the two sensor boards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard.
Both USART2 & 3 can be used for UART and I2C, PA2&3 can be used as 12bit ADCs.

## Flashing new Firmware
In order to upload the new firmware (hex file) to the microcontroller you should solder four cables into the debug pin holes (see image below) then you will need a [ST-Link programmer](https://www.ebay.co.uk/itm/ST-Link-STLink-V2-Programmer-Debugger-STM8-STM32-Blue-Pill-latest-Firmware-UK/262087067780?hash=item3d059b4884:g:TE0AAOSwAYtWGQqG)

![Alt text](debug_pins.jpeg?raw=true "Debug pins")

Then you should connect the debug pins to the pins in the ST-link according to the following table:

|  Motor Board |  ST Link V2  |
|:---:|:---:|
| SWDIO  | SWDIO  |
|  GND |  GND |
| SWCLK  |  SWCLK |
| 3.3V  | 3.3V  |
| reset pin (optional)  |  RST |


To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile. Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect these to your SWD programmer, like the ST-Link found on many STM devboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing.

Simply connect a USB-USART adapter to GND; PA2 and PA3 and give it a try.
