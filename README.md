# Analog-to-Digital-Data-Acquisition-Card
this project has two main parts:
1. ADC board
2. GUI
### ADC Board
This is an Analog to Digital Data acquisition card based on ATxmega64A3U microcontroller. Low power, high performance 8/16-bit AVR microcontroller featuring 64KB self-programming flash program memory, 4KB boot code section, 4KB SRAM, 2048-Byte EEPROM, 4-channel DMA controller, 8-channel event system, and up to 32 MIPS throughput at 32MHz. The ATxmega A3 series features 64-pin packages. it has 8 ADC with 12-bit, 2msps Analog to Digital Converters. the code of this project is developed with atmel studio 6.2. 
you can see the PCB which is designed for this board in this picture:

![Capture](https://user-images.githubusercontent.com/60741325/87361361-54ea4300-c56c-11ea-8c52-4b16cf980244.PNG)
 as we can see in the PCB it has a USB connection which is one of the peripherals of our microcontroller. it is a USB2 full speed (12Mbps). I was interested in an adjustable ADC channel for this board which can be selected by the user. In this regard, I needed a Digital potentiometers(MCP41010). you can see the schematic that is designed with this adjustable potentiometer. Single/Dual Digital Potentiometer with SPI™ Interface
 ![Capture](https://user-images.githubusercontent.com/60741325/87363174-e65bb400-c570-11ea-83bd-db589fdd308e.PNG)
 
 you can see full version of schematic of this project in this picture
 
 ![Capture](https://user-images.githubusercontent.com/60741325/87363971-af869d80-c572-11ea-92b7-6852d3b0ba32.PNG)

 ## GUI software
 The GUI part is developed based on labview software while user can control and monitor everything from this GUI. you can see this GUI in the following figure
 
 ![Capture](https://user-images.githubusercontent.com/60741325/87363676-f32cd780-c571-11ea-89d3-151f3eae458e.PNG)
 
all the data can be stored on the pc with this GUI.
