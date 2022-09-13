/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include <stdlib.h>
#include <string.h>                           //Libreri para manejo de caracteres
#include "stm32f1xx_hal_def.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;     //TIM_HandleTypeDef htim2

/* USER CODE BEGIN PV */
double pulso, frecuencia, capacitancia, inductancia, inductancia_mH;
char Convert[10];
long cont;
char flag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void Convertdat(double value, char Pos);
static void delay_us(uint32_t us);
char Fildat(char value);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
	uint16_t timer_val;
	uint16_t readValueADC;
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim2);                  // INICIA TIMER 2

    HD44780_Init(2);
    HD44780_Clear();
    HD44780_SetCursor(0,0);
    HD44780_PrintStr("FREC:        Khz");
    HD44780_SetCursor(0,1);
    HD44780_PrintStr("IND:          uH");
    HAL_Delay(2000);
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);                       // Indicamos que inicio
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);



    // ********* //

    frecuencia = 0;
    inductancia = 0;

    Convertdat(frecuencia,0);                                    // Muestra frecuencia
    Convertdat(inductancia,1);                                   // Muestra Inductancia
    char Estado = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);           /* Lee estado del Pin 1 de salida del LM393*/


    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);		/* Enviamos el pulso, quedará sostenido*/
    HAL_Delay(50);								     /* Delay 5ms para dar tiempo de carga al inductor */


    do
          {


          } while(flag != 1); /* Esto funciona hasta que el timer llegue a 65355 y marque flag en 1 */
    flag = 0;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0); /* aca se suelta el pulso para iniciar */
    delay_us(50);




    do
      {
      Estado = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);           // Lee estado del Pin 1 de salida del LM393

      } while(Estado != GPIO_PIN_RESET);


    timer_val = __HAL_TIM_GET_COUNTER(&htim2); /*Tomamos el valor que viene del timer en us */
    pulso = timer_val; /* valor del timer se lo enviamos a la variable pulso */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); /* volvemos a poner en 1 la entrada del circuito para que deje de oscilar y tomar tiempo */
    //timer_val = __HAL_TIM_GET_COUNTER(&htim2) - timer_val;



    HAL_TIM_Base_Start_IT(&htim2);                  // INICIA TIMER 2
    Convertdat(timer_val,0); /* recibe 2 parametros; el primero es el valor a convertir y el segundo es la posicion de inicio de visualizacion */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    HAL_Delay(2000);



    capacitancia = 1.2E-6; // - insert value here
     // Calculo de la Inductancia en uH y mH
    frecuencia =1E6/(2*pulso);
    inductancia = 1/(capacitancia*frecuencia*frecuencia*4.*3.14159*3.14159);//formula de la inductancia
    inductancia *= 1E6; // multiplico por 1 millon para poderla visualizar
    inductancia_mH = inductancia / 10000; //note that this is the same as saying inductance = inductance*1E6


    Convertdat(frecuencia,0);                                    // Muestra frecuencia
    Convertdat(inductancia_mH,1);                                // Muestra Inductancia


    /* USER CODE END 2 */

      /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      while (1)
    	    {
        /* USER CODE END WHILE */

    	  //HAL_TIM_Base_Start_IT(&htim2);                  // INICIA TIMER 2
    	  //flag = 0;
    	  HAL_ADC_Start(&hadc1);
    	  hadc1.Init.ContinuousConvMode = ENABLE;
    	  HAL_ADC_PollForConversion(&hadc1,1000);
    	  readValueADC = HAL_ADC_GetValue(&hadc1);
    	  //Estado = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);                 /* Lee estado del Pin de entrada de la fotorisistencia*/
    	  	  if(readValueADC < 1000)
    	   {

    	  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    	  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);


    	  	 flag = 1;

    	  	    do
    	  	          {


    	  	          } while(flag != 0);
    	  	    flag = 0;

    	  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
    	  	    delay_us(50);



    	  	    do
    	  	      {
    	  	      Estado = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);           // Lee estado del Pin 1 de salida del LM393

    	  	      } while(Estado != GPIO_PIN_RESET);


    	  	   pulso = timer_val;
    	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);



    	  	    Convertdat(timer_val,0);
    	  	   HAL_Delay(2000);
    	  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);


    	  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    	  	    HAL_Delay(2000);
    	  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);


    	  	    capacitancia = 1.2E-6; // - insert value here
    	  	     // Calculo de la Inductancia en uH y mH
    	  	    frecuencia =1E6/(2*pulso);
    	  	    inductancia = 1/(capacitancia*frecuencia*frecuencia*4.*3.14159*3.14159);//one of my profs told me just do squares like this
    	  	    inductancia *= 1E6; //note that this is the same as saying inductance = inductance*1E6
    	  	    inductancia_mH = inductancia / 10000; //note that this is the same as saying inductance = inductance*1E6

                /* Autorango Frecuencia  */
    	  	    //frecuencia=1000000;
                if(frecuencia < 1000)
                {
    	  	        HD44780_SetCursor(13,0);
    	  	        HD44780_PrintStr(" hz");

                }
                else if(frecuencia >= 1000000)
                {
                	HD44780_SetCursor(13,0);
                    HD44780_PrintStr("Mhz");
                }
                else{
                	HD44780_SetCursor(13,0);
                	HD44780_PrintStr("Khz");
                }

    	  	    Convertdat(frecuencia,0);                                    // Muestra frecuencia
    	  	    Convertdat(inductancia_mH,1);                                   // Muestra Inductancia

    	   }

    	  	//HAL_Delay(1000);
    	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);

    	}




        /* USER CODE BEGIN 3 */

      /* USER CODE END 3 */
 }






/**
  * @brief System Clock Configuration
  * @retval None
  *
  *
  *
  *
  *
  *
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*  CONVIERTE ENTERO A CARACTER */
void Convertdat(double value, char Pos)   /* Recibe el numero a convertir en caracteres para mostrarlo */
{
   Convert[0]= 0;Convert[1]= 0;Convert[2]= 0;Convert[3]= 0;Convert[4]= 0;Convert[5]= 0;
   Convert[6]= 0;Convert[7]= 0;Convert[8]= 0;Convert[9]= 0;
   itoa(value, Convert, 10);									 /* Convierte entero a un caracter y lo asigna en un arreglo */
   HD44780_SetCursor(5,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[0]));
   HD44780_SetCursor(6,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[1]));
   HD44780_SetCursor(7,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[2]));
   HD44780_SetCursor(8,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[3]));
   HD44780_SetCursor(9,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[4]));
   HD44780_SetCursor(10,Pos);
   HD44780_PrintSpecialChar(Fildat(Convert[5]));
   HD44780_SetCursor(11,Pos);

   //HD44780_PrintSpecialChar(Fildat(Convert[6]));
   //HD44780_SetCursor(12,Pos);
   //HD44780_PrintSpecialChar(Fildat(Convert[7]));
   //HD44780_SetCursor(12,Pos);
   //HD44780_PrintSpecialChar(Fildat(Convert[8]));
   //HD44780_SetCursor(14,Pos);
   //HD44780_PrintSpecialChar(Fildat(Convert[9]));


}


/*  FILTRA CARACTER CARACTER */
char Fildat(char value){

	char Valor;

	if( value > '9' || value < '0')
		   {

		Valor = ' ';

		if(value == '.'){
			Valor = value;
		    }

	  }
	else Valor = value;

   return Valor;
}





static void delay_us(uint32_t us) {
  uint32_t cycles = (SystemCoreClock/1000000L)*us;
  uint32_t start = DWT->CYCCNT;
  volatile uint32_t cnt;

  do
  {
    cnt = DWT->CYCCNT - start;
  } while(cnt < cycles);
}






void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	  {
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	    flag =1;
	  }


}







/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
