## Hoverboard-Board-Hack

This repo contains open source firmware for generic Hoverboard Mainboards.
the firmware you can find here allows you to use your Hoverboard Hardware (like the Mainboard, Motors and Battery) for cool projects like driving armchairs, person-tracking transportation robots and ervry other application you can imagine that requires controlling the Motors.

#### Hardware
![otter](https://raw.githubusercontent.com/NiklasFauth/Hoverboard-Board-Hack/master/schema.jpg)

The original Hardware supports two 4-pin cables that originally were connected to the two sensor boards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard.
Both USART2 & 3 can be used for UART and I2C, PA2&3 can be used as 12bit ADCs.

#### Flashing
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile. Right to the STM32, there is a debugging header with GND, 3V3, SWD and SCL. Connect these to your SWD programmer, like the ST-Link found on many STM devboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing.

#### Examples

There are currently 3 Branches in this repo: master, PPM and GameTrak.

##### master

Master uses UART DMA to receive two floats on USART2 with 115200 baud, and uses these floats to control the motor current of each motor. It also counts the hall ticks and sends back the motor positions to use them for odometry data, for example combined with ROS.

Simply connect a USB-USART adapter to GND; PA2 and PA3 and give it a try.

##### PPM

PPM allows you to connect a radio control with PPM sum signal output and use it to directly control and steer the motors. Channel 0 is used for motor current scaling, Ch 1&2 for speed / direction. You can change that as you want in the main.c fiile.

Connect your RC receiver to GND and 12V (if is supports 12V input voltage, else use a separate voltage regulator) and the PPM sum signal to PA3. The firmware also streams out the received channel values on USART3 / 115200 baud for debugging reasons.

This firmware also features some bare minimum protection functions like undervoltage protection (32V) and overcurrent limit (40A / motor).

##### GameTrak

This firmware allows you to connect the sensor unit of a GameTrak PS2 controller to the ADC channels PA2 & 3. Use the potentiometers for x and z as voltage dividers (3V3 VREF) and connect them to PA2 & PA3. Also a great template for your own projects that require almost all peripherals like USART & ADC readout of some type of analog sensor (like a joystick). Some protection features as PPM.
