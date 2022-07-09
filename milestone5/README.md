# MKEL1123
MKEL1123 Advanced Microprocessor System Assignment 
<br>
Milestone 5
<br>
Group 5

---
### **Overview**
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/block%20diagram.PNG" height="200px" width="250px">
</p>
	
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
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/prototype.png" width="800">
<p float="left">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/Nucleo-STM32F446RE.png" height="400px" width="400px" >
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/components.png" height="400px" width="400px" >
</p>

## **Software Installation**
:one: [STM3232CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

## System Architecture


## STM32CubeIDE
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32F446%20pinout%20view.png" height="400px" width="400px">
</p>
- PA0: ADC2_IN0 (Water Level Sensor) <br>
- PA1: ADC1_IN1 (LDR Sensor) <br>
- PB9: GPIO_Output (DHT22 Sensor) <br>
- PA5: GPIO_Output (Relay Module & Water Pump Motor) <br>
- PA9: USART1_RX (ESP8266 Wi-Fi Module) <br>
- PA10: USART1_TX (ESP8266 Wi-Fi Module) <br>
<br>

**Step :one: Import the source codes downloaded from GitHub**
<br>
:small_blue_diamond: Go to **File** then **Import**

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_import1.png" width="600">
</p>

:small_blue_diamond: Select **Existing Projects into Workspace** and **Next**

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_import2.png" width="450">
</p>

:small_blue_diamond: Browse the [source code](https://github.com/meitung/MKEL1123/tree/main/milestone5/testingThingspeak) downloaded from GitHub and **Finish**

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_import3.png" width="450">
<br>
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_import4.png" width="450">
<br>
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_import5.png" width="450">
</p>
<br>

**Step :two: Configure the setting of Wi-Fi and ThingSpeak**
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_modify1.png" width="600">
</p>

:small_blue_diamond: Insert your network credentials into the **WIFI_NAME** and **WIFI_PASSWORD** fields in line 220 of main.c

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_modify2.png" width="450">
</p>

:small_blue_diamond: Change the **WRITE_API_KEY** in line 658 of main.c to the Write API Key obtained from ThingSpeak after setting the channel
<br>
:thought_balloon: Further explanation can be found in [next section](https://github.com/meitung/MKEL1123/tree/main/milestone5#thingspeak-setup)

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_modify3.png" width="450">
</p>
<br>

**Step :three: Upload the program to your Nucleo-F446RE board**
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/STM32CubeIDE_modify4.png" width="600">
</p>
<br>

### FreeRTOS
1. xxx
2. 


### Explanation
wifi connection <br>
write api key of thingspeak


## ThingSpeak Setup
**Step :one: Regsiter and login to [ThingSpeak](https://thingspeak.com/) account**
<br>
:small_blue_diamond: For first time users, create an ThingSpeak account
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak1.png" width="550">
</p>
:small_blue_diamond: Fill up your email address, loaction and name
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak2.png" width="550">
</p>
:small_blue_diamond: Verify your registered email
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak3.png" width="400">
</p>
:small_blue_diamond: After the registration, log into ThingSpeak by entering your registered email and password
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak4.png" width="550">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak5.png" width="550">
</p>
<br>

**Step :two: Create new channel to monitor the value of the sensors**
<br>

:small_blue_diamond: Create a new channel
<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak6.png" width="550">
</p>

:small_blue_diamond: Type a name for your channel and add description (optional). Since 4 parameters are monitored, 4 fields are enabled and labelled to "Light Intensity", "Temperature", "Water Level" and "Humidty". Click the **Save Channel** button to create and save your channel.
<br>

:thought_balloon: You may add more fields if you would like to add more sensor value or change the label name to your desired name.

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak7.png" width="550">
</p>

:small_blue_diamond: To send values from Nucleo-F446RE to ThingSpeak, the **Write API Key** is required to be copied into line 658 of main.c (STM32CubeIDE). It can be found in the **API Keys** tab.

<p align="center">
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone5/Photos/thingspeak8.png" width="550">
</p>
<br>

## **Quick Links**
- [YouTube Demostration Video (Milestone 4)](https://youtu.be/L6ZIIia__Tc)
- [YouTube Progress Video (Milestone 3)](https://youtu.be/ZYLQebmYB-s)
- [Source Code](https://github.com/meitung/MKEL1123/tree/main/milestone5/testingThingspeak/Core)

### Source Code
* `main.c`
```C

#include "main.h"
#include "cmsis_os.h"

#include "UartRingbuffer.h"
#include "ESPDataLogger.h"
#include "stdio.h"
#include "stm32f4xx.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void Task2_int(void const * argument);
void Task3_int(void const * argument);
void Task4_int(void const * argument);

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_9
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT22_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
  microDelay (1300);   // wait for 1300us
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

uint16_t ADC_Get_Value(void)
{
	uint16_t val = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return val;

}

uint16_t waterLevelSensor(void)
{
	uint16_t val = 0;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	val = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	return val;

}

uint16_t AD_RES = 0, Vamb, DC_Multiplier;
uint16_t ADC_Value = 0;
uint16_t waterLevel;
uint16_t value_buf [4];

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  Vamb = HAL_ADC_GetValue(&hadc1);
  DC_Multiplier = 65535/(4096-Vamb);

  HAL_TIM_Base_Start(&htim1);
  ESP_Init("WIFI_NAME", "WIFI_PASSWORD");

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(Task2, Task2_int, osPriorityAboveNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task3, Task3_int, osPriorityNormal, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  osThreadDef(Task4, Task4_int, osPriorityNormal, 0, 128);
  Task4Handle = osThreadCreate(osThread(Task4), NULL);

  osKernelStart();

  while (1)
  {

	     }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);

}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void Task2_int(void const * argument)
{

  for(;;)
  {
	  if (DHT22_Start())
	 	      {
	 	        RH1 = DHT22_Read(); // First 8bits of humidity
	 	        RH2 = DHT22_Read(); // Second 8bits of Relative humidity
	 	        TC1 = DHT22_Read(); // First 8bits of Celsius
	 	        TC2 = DHT22_Read(); // Second 8bits of Celsius
	 	        SUM = DHT22_Read(); // Check sum
	 	        CHECK = RH1 + RH2 + TC1 + TC2;
	 	        if (CHECK == SUM)
	 	        {
	 	          if (TC1>127) // If TC1=10000000, negative temperature
	 	          {
	 	            tCelsius = (float)TC2/10*(-1);
	 	          }
	 	          else
	 	          {
	 	            tCelsius = (float)((TC1<<8)|TC2)/10;
	 	          }
	 	          tFahrenheit = tCelsius * 9/5 + 32;
	 	          RH = (float) ((RH1<<8)|RH2)/10;
	 	        }
	 	      }
	 	      HAL_Delay(1000);

	  waterLevel = waterLevelSensor();
	  ADC_Value = ADC_Get_Value();

	  value_buf[0] = ADC_Value;
	  value_buf[1] = tCelsius;
	  value_buf[2] = waterLevel;
	  value_buf[3] = RH;

	  ESP_Send_Multi("WRITE_API_KEY", 4, value_buf);

	  HAL_Delay(15);
    osDelay(1);
  }
  
}

void Task3_int(void const * argument)
{
  for(;;)
  {

	AD_RES = HAL_ADC_GetValue(&hadc1);
	TIM2->CCR1 = (AD_RES-Vamb)*DC_Multiplier;

    osDelay(1);
  }
}

void Task4_int(void const * argument)
{
  for(;;)
  {
	  if (tCelsius > 32){
		  if (RH < 80){
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			  HAL_Delay(10000); //motor run for 10s (for demo only)
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			  HAL_Delay(10000); //motor stop for 10s
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			  HAL_Delay(8000); //motor run for 8s (for demo only)
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			  HAL_Delay(10000); //motor stop for 10s
		  }
	  }
	  else {
		  if (RH < 80){
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			  HAL_Delay(6000); //motor run for 6s (for demo only)
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			  HAL_Delay(10000); //motor stop for 10s
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			  HAL_Delay(4000); //motor run for 4s (for demo only)
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			  HAL_Delay(10000); //motor stop for 10s

		  }

	  }
    osDelay(1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }

}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif 

```
### Reference
- [Introduction to FreeRTOS](https://controllerstech.com/introduction-to-free-rtos-in-stm32/)
- [DHT22 Tutorial Video](https://www.youtube.com/watch?v=zuvvzTh4d4E&ab_channel=NizarMohideen-MicroPeta)
- [DHT22 in STM32](https://www.micropeta.com/video48)
- [LDR Sensor in STM32](https://deepbluembedded.com/stm32-light-sensor-ldr-interfacing-ambient-light-sensor-project/)
- [ThingSpeak in STM32](https://controllerstech.com/data-logger-using-stm32-and-esp8266)
- [STM32 Nucleo-64 Boards User Manual](https://drive.google.com/file/d/1GAqdJ5bWztGX7JlX7BPyD6pmOoSrM1mf/view?usp=sharing)
- [Description of STM32F4 HAL and Low-Layer Drivers](https://drive.google.com/file/d/1y4wEi0xtDwZTLbO_yoIVwKH5LEFnJ6sZ/view?usp=sharing)
