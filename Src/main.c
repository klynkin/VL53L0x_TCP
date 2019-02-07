
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
VL53L0X_Dev_t	myDevStruct[3];
VL53L0X_DEV myDev;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
 // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_UART_Transmit(&huart2, "allrightt\t\n", strlen("allright\t\n"), 0xffff);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  HAL_Delay(5);
  //-----------------------------------------------------------------------------------------------------------------------------------------------
  int k;
  for (k=0;k<=2;k++) {

  myDev=&myDevStruct[k];
  myDev->I2cDevAddr=0x52;
  myDev->Present = 0;
  myDev->I2cHandle=&hi2c2;

  	uint8_t str[100];
      int status;
      uint16_t Id;
  		status = VL53L0X_RdWord(myDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
  		if (!status)
  		{
  		sprintf(str, "%d my id \r\n", Id);
  		HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  		}
  		else
  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
  		HAL_Delay(10);
  		VL53L0X_ResetDevice(myDev);
  		status = VL53L0X_SetDeviceAddress(myDev, 0x52+(k+1)*2);
                  if( status == 0 ){
  								myDev->I2cDevAddr=0x52+(k+1)*2;
  								sprintf(str, "Adress ok  %x\r\n", myDev->I2cDevAddr);
  								HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
                  }
                  else
  									{

                	  	  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  							sprintf(str, "Adres fail %d\r\n", status);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
                    }

  									HAL_Delay(10);

                  status = VL53L0X_DataInit(myDev);
  								if( status == 0 ){
                      myDev->Present = 1;
                  }
                  else
  									{
  								sprintf(str, "Data init fail %d\r\n", status);
  								HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

                    }
  									HAL_Delay(20);


    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
  	uint8_t isApertureSpads;
  	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
  	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
  //-----------------------------------------------------------------------------------------------------------------------------------------------

  	uint32_t timingBudget = 200000;
  	uint8_t preRangeVcselPeriod = 14;
  	uint8_t finalRangeVcselPeriod = 10;
  signalLimit = (FixPoint1616_t)(0.1*65536);
  sigmaLimit = (FixPoint1616_t)(60*65536);
  timingBudget = 33000;
  preRangeVcselPeriod = 18;
  finalRangeVcselPeriod = 14;


 //-----------------------------------------------------------------------------------------------------------------------------------------------

              status=VL53L0X_StaticInit(myDev);
              if( status ){
  							sprintf(str, "static init fail %d\r\n", status);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  					  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  							}
  								else {
  									sprintf(str, "static init Ok\r\n");
  									 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  								}
  			HAL_Delay(1);


  						status = VL53L0X_PerformRefSpadManagement(myDev, &refSpadCount, &isApertureSpads);
  					if( status ){
  						sprintf(str, "perform SPAD calibration fail %d\r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  						}
  								else {
  										sprintf(str, "spad calibration Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}

  									HAL_Delay(10);


              status = VL53L0X_PerformRefCalibration(myDev, &VhvSettings, &PhaseCal);
  						if( status ){
  						sprintf(str, "Ref calibration fail %d\r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  							}
  								else {
  												sprintf(str, "Ref calibration ok \r\n", status);
  												 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  											}
  												HAL_Delay(1);

  	 //-----------------------------------------------------------------------------------------------------------------------------------------------


              status = VL53L0X_SetLimitCheckEnable(myDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
  			if( status ){
  			   sprintf(str, "SetLimitCheckEnable fail			%d\r\n", status);
  					HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetLimitCheckEnable Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}
  				HAL_Delay(1);


  			status = VL53L0X_SetLimitCheckEnable(myDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
  			if( status ){
  				sprintf(str, "SetLimitCheckEnable signal fail\r\n");
  				HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetLimitCheckEnable signal Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}
  				HAL_Delay(1);


  	status = VL53L0X_SetLimitCheckValue(myDev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
  			if( status ){
  			   sprintf(str, "SetLimitCheckValue signal fail\r\n");
  			   HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  			   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetLimitCheckValue signal Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}

  				HAL_Delay(1);


  			status = VL53L0X_SetLimitCheckValue(myDev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
  			if( status ){
  			   sprintf(str, "SetLimitCheckValue sigma fail\r\n");
  			   HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
    		   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetLimitCheckValue sigma Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}

  				HAL_Delay(1);


              status = VL53L0X_SetVcselPulsePeriod(myDev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
  			if( status ){
  			   sprintf(str, "SetVcselPulsePeriod fail\r\n");
  			   HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  			   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetVcselPulsePeriod Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}
  				HAL_Delay(1);


              status = VL53L0X_SetVcselPulsePeriod(myDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
  			if( status ){
  				sprintf(str, "SetVcselPulsePeriod final fail\r\n");
  				HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  					}
  								else {
  										sprintf(str, "SetVcselPulsePeriod final Ok\r\n");
  										 HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  									}
  				HAL_Delay(1);


  					status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(myDev, timingBudget);

  	 //-----------------------------------------------------------------------------------------------------------------------------------------------

  			VL53L0X_SetDeviceMode(myDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  			VL53L0X_SetInterruptThresholds(myDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ,  200<<16 ,  0<<16);
        status = VL53L0X_SetGpioConfig(myDev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW, VL53L0X_INTERRUPTPOLARITY_HIGH);
  	 //-----------------------------------------------------------------------------------------------------------------------------------------------

  		VL53L0X_StopMeasurement(myDev);  // it is safer to do this while sensor is stopped
if(k==0)
  {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  }
else{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}
  											HAL_Delay(75);

  							}
    //  status = VL53L0X_ClearInterruptMask(myDev, -1); // clear interrupt pending if any
  myDev=&myDevStruct[0];
   			VL53L0X_StartMeasurement(myDev);
   			VL53L0X_ClearInterruptMask(myDev, -1);
   		HAL_Delay(10);
  myDev=&myDevStruct[1];
   			VL53L0X_StartMeasurement(myDev);
   			VL53L0X_ClearInterruptMask(myDev, -1);
   		HAL_Delay(10);
  myDev=&myDevStruct[2];
     		VL53L0X_StartMeasurement(myDev);
     		VL53L0X_ClearInterruptMask(myDev, -1);
     		HAL_Delay(10);
  	 //-----------------------------------------------------------------------------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
