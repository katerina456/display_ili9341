/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ili9341.h"
#include "fonts.h"
#include "testimg.h"
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  void greeting_init() {
	  ILI9341_FillScreen(ILI9341_BLACK);

	        for(int x = 0; x < ILI9341_WIDTH; x++) {
	            ILI9341_DrawPixel(x, 0, ILI9341_RED);
	            ILI9341_DrawPixel(x, ILI9341_HEIGHT-1, ILI9341_RED);
	        }

	        for(int y = 0; y < ILI9341_HEIGHT; y++) {
	            ILI9341_DrawPixel(0, y, ILI9341_RED);
	            ILI9341_DrawPixel(ILI9341_WIDTH-1, y, ILI9341_RED);
	        }

	        HAL_Delay(500);

	        ILI9341_WriteString(2, 72, "Press the button to see variants", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
  }

  void init() {
      ILI9341_Unselect();
  //    ILI9341_TouchUnselect();
      ILI9341_Init();

      greeting_init();
  }



  void loop() {
    /*  if(HAL_SPI_DeInit(&hspi1) != HAL_OK) {
     //     UART_Printf("HAL_SPI_DeInit failed!\r\n");
          return;
      }

      hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;

      if(HAL_SPI_Init(&hspi1) != HAL_OK) {
       //   UART_Printf("HAL_SPI_Init failed!\r\n");
          return;
      }*/

      // Check border
      ILI9341_FillScreen(ILI9341_BLACK);

      for(int x = 0; x < ILI9341_WIDTH; x++) {
          ILI9341_DrawPixel(x, 0, ILI9341_RED);
          ILI9341_DrawPixel(x, ILI9341_HEIGHT-1, ILI9341_RED);
      }

      for(int y = 0; y < ILI9341_HEIGHT; y++) {
          ILI9341_DrawPixel(0, y, ILI9341_RED);
          ILI9341_DrawPixel(ILI9341_WIDTH-1, y, ILI9341_RED);
      }

      HAL_Delay(500);

      ILI9341_WriteString(2, 76, "Press the button to see all images", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);

      HAL_Delay(3000);

      // Check fonts
      ILI9341_FillScreen(ILI9341_BLACK);
      ILI9341_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ILI9341_RED, ILI9341_BLACK);
      ILI9341_WriteString(0, 3*10, "Font_11x18, green, lorem ipsum dolor sit amet", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
      ILI9341_WriteString(0, 3*10+3*18, "Font_16x26, blue, lorem ipsum dolor sit amet", Font_16x26, ILI9341_BLUE, ILI9341_BLACK);

      HAL_Delay(1000);
      ILI9341_InvertColors(true);
      HAL_Delay(1000);
      ILI9341_InvertColors(false);

      HAL_Delay(5000);

      // Check colors
      ILI9341_FillScreen(ILI9341_WHITE);
      ILI9341_WriteString(0, 20, "WHITE", Font_11x18, ILI9341_BLACK, ILI9341_RED);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_BLUE);
      ILI9341_WriteString(0, 0, "BLUE", Font_11x18, ILI9341_BLACK, ILI9341_BLUE);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_RED);
      ILI9341_WriteString(0, 0, "RED", Font_11x18, ILI9341_BLACK, ILI9341_RED);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_GREEN);
      ILI9341_WriteString(0, 0, "GREEN", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_CYAN);
      ILI9341_WriteString(0, 0, "CYAN", Font_11x18, ILI9341_BLACK, ILI9341_CYAN);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_MAGENTA);
      ILI9341_WriteString(0, 0, "MAGENTA", Font_11x18, ILI9341_BLACK, ILI9341_MAGENTA);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_YELLOW);
      ILI9341_WriteString(0, 0, "YELLOW", Font_11x18, ILI9341_BLACK, ILI9341_YELLOW);
      HAL_Delay(500);

      ILI9341_FillScreen(ILI9341_BLACK);
      ILI9341_WriteString(0, 0, "BLACK", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
      HAL_Delay(500);

      ILI9341_DrawImage((ILI9341_WIDTH - 120) / 2, (ILI9341_HEIGHT - 64) / 2, 120, 60, (const uint16_t*)test_img_240x240);

      HAL_Delay(5000);

      ILI9341_FillScreen(ILI9341_BLACK);
      ILI9341_WriteString(0, 0, "Touchpad test.  Draw something!", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
      HAL_Delay(1000);
      ILI9341_FillScreen(ILI9341_BLACK);

   /*   if(HAL_SPI_DeInit(&hspi1) != HAL_OK) {
     //     UART_Printf("HAL_SPI_DeInit failed!\r\n");
          return;
      }*/

      hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;

     /* if(HAL_SPI_Init(&hspi1) != HAL_OK) {
       //   UART_Printf("HAL_SPI_Init failed!\r\n");
          return;
      }*/

    //  int npoints = 0;
     /* while(npoints < 10000) {
          uint16_t x, y;

       /*   if(ILI9341_TouchGetCoordinates(&x, &y))
          {
              ILI9341_DrawPixel(x, 320-y, ILI9341_WHITE);
              npoints++;
          }

      }*/
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  init();

  void handler_click(int num) {
	  switch ( num ) {
	  case 1:
	    // Check fonts
        ILI9341_FillScreen(ILI9341_BLACK);
        ILI9341_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ILI9341_RED, ILI9341_BLACK);
        ILI9341_WriteString(0, 3*10, "Font_11x18, green, lorem ipsum dolor sit amet", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
        ILI9341_WriteString(0, 3*10+3*18, "Font_16x26, blue, lorem ipsum dolor sit amet", Font_16x26, ILI9341_BLUE, ILI9341_BLACK);

        HAL_Delay(1000);
        ILI9341_InvertColors(true);
        HAL_Delay(1000);
        ILI9341_InvertColors(false);
	    break;
	  case 2:
	    // Check colors
        ILI9341_FillScreen(ILI9341_WHITE);
        ILI9341_WriteString(0, 20, "WHITE", Font_11x18, ILI9341_BLACK, ILI9341_RED);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_BLUE);
        ILI9341_WriteString(0, 0, "BLUE", Font_11x18, ILI9341_BLACK, ILI9341_BLUE);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_RED);
        ILI9341_WriteString(0, 0, "RED", Font_11x18, ILI9341_BLACK, ILI9341_RED);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_GREEN);
        ILI9341_WriteString(0, 0, "GREEN", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_CYAN);
        ILI9341_WriteString(0, 0, "CYAN", Font_11x18, ILI9341_BLACK, ILI9341_CYAN);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_MAGENTA);
        ILI9341_WriteString(0, 0, "MAGENTA", Font_11x18, ILI9341_BLACK, ILI9341_MAGENTA);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_YELLOW);
        ILI9341_WriteString(0, 0, "YELLOW", Font_11x18, ILI9341_BLACK, ILI9341_YELLOW);
        HAL_Delay(500);

        ILI9341_FillScreen(ILI9341_BLACK);
        ILI9341_WriteString(0, 0, "BLACK", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
        HAL_Delay(500);
	    break;
	  case 3:
	    // Check image
		ILI9341_FillScreen(ILI9341_BLACK);
		ILI9341_DrawImage((ILI9341_WIDTH - 120) / 2, (ILI9341_HEIGHT - 64) / 2, 120, 60, (const uint16_t*)test_img_240x240);
		HAL_Delay(500);
	    break;
	  }
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int flag = 0;
  while (1)
  {
	 // loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) ) {
		flag ++;
			if (flag == 4) {
				flag = 1;
			}

		handler_click(flag);
		HAL_Delay(200);
		greeting_init();
	}


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
