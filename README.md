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
```
$ make release
$ make debug (for debug purposes)
$ cd build/Release
$ make install
$ ldconfig (updates the dynamic library cache, fixes shared library bug)
```

5. Permissions with udev, The rules are located in the etc/udev/rules.d directory. You will need to copy it to /etc/udev/rules.d and run:

```
$ udevadm control –reload-rules
$ udevadm trigger
```

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

Once you connect your ST-Link programmer to your computer you should be able to execute the following command in a terminal:
```
$ st-info --probe
```
And you should get somthing similar to the following:

```
Found 1 stlink programmers
 serial: 563f7206513f52504832153f
openocd: "\x56\x3f\x72\x06\x51\x3f\x52\x50\x48\x32\x15\x3f"
  flash: 262144 (pagesize: 2048)
   sram: 65536
 chipid: 0x0414
  descr: F1 High-density device
```
To upload the hex file you will have to type:

```
$ st-flash write file.hex 0x8000000
```

If everything is fine you should get the following lines at the end:

```
INFO src/common.c: Starting verification of write complete
INFO src/common.c: Flash written and verified! jolly good!
```
