# MKEL1123
MKEL1123 Advanced Microprocessor System Assignment 
<br>
Milestone 1
<br>
Group 5

---
### **Overview**
We are using the STM32 board. 
The STM 32 is based on the ARM Cortex M4 architecture. 
The simple C program that blinks an LED is written by using STM32CubeIDE.
The built-in LED on STM32 board is blinking at 1Hz.
In other words, the LED is blinking for every 1s or 1000ms.


---
### **Equipment Required**
:one: STM32F446RET6 (Nucleo-F446RF) <br>
:two: Personal computer <br>
:three: USB cables

### **Photo of Board**
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone1/Photo%20of%20Board/Front%20of%20Board.jpg" height="400px" width="400px" >
<img src="https://github.com/meitung/MKEL1123/blob/main/milestone1/Photo%20of%20Board/Back%20of%20Board.jpg" height="400px" width="400px" >

### **Software Installation**
:one: [STM3232CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

### **Quick Links**
- [YouTube Demostration Video](https://youtu.be/cHShAr0WdlQ)
- [Plagiarism Declaration Form](https://drive.google.com/file/d/1Plq23TwlS19BEvZTGlQb8P0T4vDp0Bwa/view)
- [Source Code](https://github.com/meitung/MKEL1123/tree/main/milestone1/milestone_1_LED_blink)

### Source Code
* `main.c`
```C
#include "main.h"

 UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();

  while (1)
  {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
   HAL_Delay(1000);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
   HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
}


```
### Reference
- [YouTube Referred Tutorial](https://youtu.be/hyZS2p1tW-g)
- [STM32 Nucleo-64 Boards User Manual](https://drive.google.com/file/d/1GAqdJ5bWztGX7JlX7BPyD6pmOoSrM1mf/view?usp=sharing)
- [Description of STM32F4 HAL and Low-Layer Drivers](https://drive.google.com/file/d/1y4wEi0xtDwZTLbO_yoIVwKH5LEFnJ6sZ/view?usp=sharing)
