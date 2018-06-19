## Hoverboard-Board-Hack

This repo contains open source firmware for generic Hoverboard Mainboards.
the firmware you can find here allows you to use your Hoverboard Hardware (like the Mainboard, Motors and Battery),

This project uses UART DMA to receive two floats on USART2 with 115200 baud, and uses these floats to control the motor current of each motor. It also counts the hall ticks and sends back the motor positions to use them for odometry data, for example combined with ROS.

#### Hardware
![alt text](https://raw.githubusercontent.com/NiklasFauth/Hoverboard-Board-Hack/master/schema.jpg)

The original Hardware supports two 4-pin cables that originally were connected to the two sensor boards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard.
Both USART2 & 3 can be used for UART and I2C, PA2&3 can be used as 12bit ADCs.

---

#### Flashing
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile. Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect these to your SWD programmer, like the ST-Link found on many STM devboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing.

Simply connect a USB-USART adapter to GND; PA2 and PA3 and give it a try.
