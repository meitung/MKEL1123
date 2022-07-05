# MKEL1123
MKEL1123 Advanced Microprocessor System Assignment 
<br>
Milestone 5
<br>
Group 5

---
### **Overview**
The objective of the proposed system is to build a low power and sustainable hydroponic farming system which offers real time monitoring and control over the important parameters. <br> <br>
STM32F446 is chosen as the microprocessor of the proposed system.  <br> <br>
To monitor water level of the tank, ambient temperature, humidity and light intensity, water level sensor, DHT22 sensor and LDR sensor are used as the input of the STM32F466 microcontroller. A water pump motor and LEDs act as the output response which are controlled by the STM32F446 microcontroller based on the input data of the DHT22 sensor and LDR sensor respectively. ESP8266 Wi-Fi Module is used for the purpose of data transfer between the cloud and the microcontroller and ThingSpeak is chosen as IoT platform. <br> <br>
The program of the microcontroller is written using the software STM32CubeIDE. To perform and execute concurrent tasks at predetermined priority, Real-Time Operating System (RTOS) is implemented and FreeRTOS is enabled in STM32CubeIDE. 

---
### **Equipment Required**
:one: [STM32F446RET6 (Nucleo-F446RE)](https://www.digikey.my/en/products/detail/stmicroelectronics/NUCLEO-F446RE/5347712?utm_adgroup=Products&utm_source=google&utm_medium=cpc&utm_campaign=Smart%20Shopping_DK%2B%20Supplier_Suntsu&utm_term=&productid=5347712&gclid=CjwKCAjwwo-WBhAMEiwAV4dybf9QIqFgZyiYbgBGT-ySye7BC3D3rw205yHemeCn94EpIbqXlbZRKhoCvn4QAvD_BwE) <br>
:two: [ESP8266 Wi-Fi Module](https://shopee.com.my/ESP-01S-ESP8266-serial-WIFI-Wireless-Transceiver-Modele--ESP-01-Updated-version--i.110910897.6806457597?gclid=CjwKCAjwwo-WBhAMEiwAV4dybflMBCLRnwpX7L79gzdY8zQ8k9z_JAhwN3UGM_yNL-7oyDT5Mdr15BoCj7AQAvD_BwE) <br>
:three: [Water Level Sensor](https://shopee.com.my/Water-Liquid-Level-Sensor-Module-for-Arduino-i.23949362.856353446?sp_atk=4f7069ff-60fb-4140-b833-573b99ddaedb&xptdk=4f7069ff-60fb-4140-b833-573b99ddaedb) <br>
:four: [DHT22 Sensor](https://shopee.com.my/DHT11-DHT22-DHT-11-22-High-Accuracy-Temperature-and-Humidity-Moisture-Sensor-3.3V-5V-Module-FREE-CABLE-for-Arduino-i.33091591.572008040?sp_atk=f6591cbd-21f8-475f-b1c6-903565792b8c&xptdk=f6591cbd-21f8-475f-b1c6-903565792b8c) <br>
:five: [LDR sensor](https://shopee.com.my/Photo-Resistor-LDR-Light-Sensor-Module-(-Light-Dependent-Resistor-)-i.33287405.463948703?sp_atk=97c9e823-7b1f-4cc3-af84-5fbd4eb704dd&xptdk=97c9e823-7b1f-4cc3-af84-5fbd4eb704dd) <br>
:six: [1-Channel 5V Relay Module](https://shopee.com.my/5V-12V-1-2-4-8-Ways-Channels-Opto-isolator-Isolated-Optocoupler-Trigger-Switch-Relay-Module-AC-240V-DC-30V-for-Arduino-i.33091591.468240805?sp_atk=6fe489ef-0f62-4fc3-929d-ab59c5aa2d3a&xptdk=6fe489ef-0f62-4fc3-929d-ab59c5aa2d3a) <br>
:seven: [Water Pump Motor](https://shopee.com.my/product/238148246/5641804433?smtt=0.28541998-1654091074.3) <br>
:eight: [12V Power Supply](https://shopee.com.my/product/238148246/5641804433?smtt=0.28541998-1654091074.3) <br>
:nine: LEDs <br>

## **Gallery**
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/prototype.png">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/Nucleo-STM32F446RE.png" height="400px" width="400px" >
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/components.png" height="400px" width="400px" >

## **Software Installation**
:one: [STM3232CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)


## System Architecture

## STM32CubeIDE
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32F446%20pinout%20view.png">
## ThingSpeak Setup

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
