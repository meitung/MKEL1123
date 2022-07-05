# MKEL1123
MKEL1123 Advanced Microprocessor System Assignment 
<br>
Milestone 5
<br>
Group 5

---
### **Overview**
The objective of the proposed system is to build a low power and sustainable hydroponic farming system which offers real time monitoring and control over the important parameters. 
STM32F446 is chosen as the microprocessor of the proposed system.  
To monitor water level of the tank, ambient temperature, humidity and light intensity, water level sensor, DHT22 sensor and LDR sensor are used as the input of the STM32F466 microcontroller. A water pump motor and LEDs act as the output response which are controlled by the STM32F446 microcontroller based on the input data of the DHT22 sensor and LDR sensor respectively. ESP8266 Wi-Fi Module is used for the purpose of data transfer between the cloud and the microcontroller and ThingSpeak is chosen as IoT platform. 
The program of the microcontroller is written using the software STM32CubeIDE. To perform and execute concurrent tasks at predetermined priority, Real-Time Operating System (RTOS) is implemented and FreeRTOS is enabled in STM32CubeIDE. 

---
### **Equipment Required**
:one: STM32F446RET6 (Nucleo-F446RF) <br>
:two: ESP8266 <br>
:three: 1-channel 5V Relay Module <br>
:four: Water Level Sensor <br>
:five: DHT22 <br>
:six: LDR sensor <br>
:seven: Water Pump Motor <br>
:eight: LEDs <br>

### **Photo of Board**
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone1/Photo%20of%20Board/Front%20of%20Board.jpg" height="400px" width="400px" >
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone1/Photo%20of%20Board/Back%20of%20Board.jpg" height="400px" width="400px" >

### **Software Installation**
:one: [STM3232CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

### ThingSpeak Setup

### **Quick Links**
- [YouTube Demostration Video](https://youtu.be/L6ZIIia__Tc)
- [YouTube Progress Video (Milestone 3)](https://youtu.be/ZYLQebmYB-s)
- [Source Code](https://github.com/meitung/MKEL1123/tree/main/milestone5/testingThingspeak/Core)

### Source Code
* `main.c`
```C
#include "main.h"



```
### Reference
- [Introduction to FreeRTOS](https://controllerstech.com/introduction-to-free-rtos-in-stm32/)
- [DHT22 Tutorial Video](https://www.youtube.com/watch?v=zuvvzTh4d4E&ab_channel=NizarMohideen-MicroPeta)
- [DHT22 in STM32](https://www.micropeta.com/video48)
- [LDR Sensor in STM32](https://deepbluembedded.com/stm32-light-sensor-ldr-interfacing-ambient-light-sensor-project/)
- [ThingSpeak in STM32](https://controllerstech.com/data-logger-using-stm32-and-esp8266)
- [STM32 Nucleo-64 Boards User Manual](https://drive.google.com/file/d/1GAqdJ5bWztGX7JlX7BPyD6pmOoSrM1mf/view?usp=sharing)
- [Description of STM32F4 HAL and Low-Layer Drivers](https://drive.google.com/file/d/1y4wEi0xtDwZTLbO_yoIVwKH5LEFnJ6sZ/view?usp=sharing)
