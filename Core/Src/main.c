/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t GAUGE_ADDR=0x64<<1;
static const uint8_t CTRL_REG=0x01; //register
static const uint8_t AUTO_MODE=0xF8; //data + prescaler à 1024
static const uint8_t SHUTDOWN=0xF9; //shutdown au registre B sans écraser
static const uint8_t VOLT_REG_MSB=0x08;
static const uint8_t VOLT_REG_LSB=0x09;
static const uint8_t AMP_REG_MSB=0x0E;
static const uint8_t AMP_REG_LSB=0x0F;
static const uint8_t COUL_REG_MSB=0x02;
static const uint8_t COUL_REG_LSB=0x03;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
HAL_StatusTypeDef Write_Register8(uint8_t register_pointer, uint8_t register_value);
HAL_StatusTypeDef Read_Register(uint8_t register_pointer, uint8_t* receive_buffer,int num_reg);
uint16_t read_register(uint8_t register_pointer);
void SetSOC(uint8_t valMSB,uint8_t valLSB);
uint16_t read_registerSOC(uint8_t REG_MSB, uint8_t REG_LSB);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	float voltage;
	float current;
	int32_t current_int;
	float SOC;
	float DeltaQ;
	float SOCmax=17; //Ah - SOC max de la batterie
	float SOCpart;
	float qLSB=0.34*(50/6.08);
	uint8_t bufRx[3];
	uint16_t bufRx16;
	uint8_t msg[50];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  while (HAL_I2C_IsDeviceReady(&hi2c1,GAUGE_ADDR,1,100)!=HAL_OK)
  {
	  strcpy((char*)msg,"I2C Device not ready.\r\n");
	  HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	  HAL_Delay(500);
  }


  	ret=Write_Register8(CTRL_REG,AUTO_MODE); //I2C Master Transmit
  	if (ret!=HAL_OK)
  	{
  		strcpy((char*)msg,"Control Register Automatic Mode Error.\r\n");
  		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
  	}
  	else //à partir de là on devrait avoir configurer le Control Register en Automatic Control, pour vérifer :
  	{
  		ret=Read_Register(CTRL_REG,bufRx,2); //attention au changement ici si PB
  		if (ret!=HAL_OK)
  		{
  			strcpy((char*)msg,"Automatic Mode Set Control Error.\r\n");
  			HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
  		}
  		else
  		{
  			strcpy((char*)msg,"Automatic Mode Set Success.\r\n");
  			HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
  		}
  	}


  	SetSOC(0x10,0x4E); //Valeur init à ...Ah ; Calcul : [Valeur/DeltaQ]_16 /!\ ATTENTION IL FAUT INVERSER MSB ET LSB : par exemple pour 0x44EF il faut mettre 0xEF et 0x44
  	HAL_Delay(500);


  	bufRx16=read_register(COUL_REG_MSB);

  	SOC=(bufRx16*qLSB)/1000.0;

  	sprintf((char*)msg,"Init SOC=%f",SOC);
  	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
  	HAL_Delay(50);
  	strcpy((char*)msg," Ah\r\n");
  	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
  	HAL_Delay(1000);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//VOLTAGE READ ->
	bufRx16=read_registerSOC(VOLT_REG_MSB,VOLT_REG_LSB);
	voltage=70.8*(bufRx16/65535.0);
	sprintf((char*)msg,"Voltage=%f V\r\n",voltage);
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	HAL_Delay(50);


	//CURRENT READ ->
	bufRx16=read_registerSOC(AMP_REG_MSB,AMP_REG_LSB);
	current=64.0*((float)(bufRx16-32767)/(float)(32767*6.08));
	//current=(current^2)^0.5;
	sprintf((char*)msg,"Current=%f A\r\n",current);
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	HAL_Delay(50);


	//SOC
	bufRx16=read_registerSOC(COUL_REG_MSB,COUL_REG_LSB); //A TESTER - attention ce n'est pas la même dans l'init

	//DeltaQ=(bufRx16-0x4E10)*qLSB; //(ACR-qOffset)*qLSB;
	SOC=(bufRx16*qLSB)/1000.0;

	sprintf((char*)msg,"SOC=%f Ah (/%f Ah)\r\n",SOC,SOCmax);
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	HAL_Delay(50);
	//calcul SOC pourcentage
	SOCpart=100*SOC/SOCmax;
	sprintf((char*)msg,"SOC=%f %% \r\n\n",SOCpart);
	HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	HAL_Delay(50);



	HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}




void SetSOC(uint8_t valMSB,uint8_t valLSB)
{
	HAL_StatusTypeDef ret;
	uint8_t msg[50];

	ret=Write_Register8(CTRL_REG,SHUTDOWN);
	if (ret==HAL_OK)
	{
		HAL_Delay(100);
		Write_Register8(COUL_REG_MSB,valMSB);
		ret=Write_Register8(COUL_REG_LSB,valLSB);
		if (ret==HAL_OK)
		{
			HAL_Delay(100);
			ret=Write_Register8(CTRL_REG,AUTO_MODE);
			if (ret==HAL_OK)
			{
				HAL_Delay(100);
				strcpy((char*)msg,"Setting Previous SOC Success.\r\n");
				HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
			}
			else
			{
				strcpy((char*)msg,"Control Register Still Shutdown.\r\n");
				HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
			}
		}
		else
		{
			strcpy((char*)msg,"ERROR: not setting previous SOC value.\r\n");
			HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
		}
	}
	else
	{
		strcpy((char*)msg,"Control Register Not Shutdown - Retry.\r\n");
		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	}
}


HAL_StatusTypeDef Write_Register8(uint8_t register_pointer, uint8_t register_value)
{
	uint8_t data[2];
	data[0]=register_pointer;
	data[1]=register_value;

	return HAL_I2C_Master_Transmit(&hi2c1,GAUGE_ADDR,data,2,HAL_MAX_DELAY);
}

//A retirer - NE PAS PRENDRE EN COMPTE POUR LE CODE FINAL - pourquoi pas le faire en deux fois ?
HAL_StatusTypeDef Read_Register(uint8_t register_pointer, uint8_t* receive_buffer,int num_reg) //num_reg est le nombre d'octet à récupérer (généralement 1 ou 2)
{
	HAL_I2C_Master_Transmit(&hi2c1, GAUGE_ADDR, &register_pointer,1,HAL_MAX_DELAY);
	return HAL_I2C_Master_Receive(&hi2c1, GAUGE_ADDR, receive_buffer,num_reg,HAL_MAX_DELAY);
}


uint16_t read_register(uint8_t register_pointer)
{
	HAL_StatusTypeDef ret;
	uint16_t return_value=0;
	uint8_t msg[20];

	ret=HAL_I2C_Mem_Read(&hi2c1,(uint16_t)GAUGE_ADDR,(uint16_t)register_pointer,I2C_MEMADD_SIZE_8BIT,&return_value,2,HAL_MAX_DELAY);
	if (ret!=HAL_OK)
	{
		strcpy((char*)msg,"Read Error.\r\n");
		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	}
	return return_value;
}


uint16_t read_registerSOC(uint8_t REG_MSB, uint8_t REG_LSB)
{
	HAL_StatusTypeDef ret;
	uint16_t retMSB,retLSB;
	uint8_t msg[20];

	ret=HAL_I2C_Mem_Read(&hi2c1,(uint16_t)GAUGE_ADDR,(uint16_t)REG_MSB,I2C_MEMADD_SIZE_8BIT,&retMSB,1,HAL_MAX_DELAY);
	if (ret!=HAL_OK)
	{
		strcpy((char*)msg,"Read Error.\r\n");
		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	}
	retMSB=retMSB<<8;

	ret=HAL_I2C_Mem_Read(&hi2c1,(uint16_t)GAUGE_ADDR,(uint16_t)REG_LSB,I2C_MEMADD_SIZE_8BIT,&retLSB,1,HAL_MAX_DELAY);
	if (ret!=HAL_OK)
	{
		strcpy((char*)msg,"Read Error.\r\n");
		HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
	}

	return retMSB+retLSB;
}





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
