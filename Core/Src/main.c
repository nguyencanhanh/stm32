#include "main.h"
#include "stdio.h"

int checkFire, x;
uint16_t ADC_ScanVal[4];

/* Declare variable */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

/* Delay */
void Delay_us(uint16_t us){
	TIM1->CNT = 0;
	while(TIM1->CNT < us);
}
void Delay_ms(uint16_t ms){
	while(ms){
		Delay_us(1000);
		ms--;
	}
}

/* Enable Buzzer while button is pushed */
void on_voice(void){
	if(!(GPIOB->IDR & (1<<6))){
		GPIOB->BSRR |= GPIO_PIN_14;     
		Delay_ms(2000);
		GPIOB->BRR |= GPIO_PIN_14;
	}
}

/*  UART */
void UART_sendData(char* str){
	while(*str){
		while((USART1->SR & (1<<7)) == 0);
		USART1->DR = *str;
		str++;
	}
	while((USART1->SR & (1<<6)) == 0);
}

void UART_sendFloat(float f){
	char str[9];
	int phanNguyen = (int)f;
	int phanThapPhan = (int)((f-phanNguyen)*100);
	sprintf(str, "%d.%.2d", phanNguyen, phanThapPhan);
	UART_sendData(str);
}

/* DHT22 */
void DHT22_Start(void){
	GPIOB->BRR |= GPIO_PIN_5;
	Delay_ms(20);
	GPIOB->BSRR |= GPIO_PIN_5;
	TIM1->CNT = 0;
	while(TIM1->CNT < 55){
		if(!(GPIOB->IDR & (1<<5)))
			break;
	}
}

uint8_t DHT22_checkResponse(void){
	uint8_t res = 0;
	TIM1->CNT = 0;
	while(TIM1->CNT < 90){
		if((GPIOB->IDR & (1<<5)) && (TIM1->CNT > 70)){
			res++;
			break;
		}
	}

	TIM1->CNT = 0;
	while(TIM1->CNT <95){
		if(!(GPIOB->IDR & (1<<5)) && (TIM1->CNT > 70)){
			res++;
			break;
		}
	}
	return res;
}

void DHT22_readData(float *temp, float *hum){
	uint8_t data[5];
	DHT22_Start();
	uint8_t check = DHT22_checkResponse();
	if(check == 2){
		for(int i = 0; i < 5; i++){
			for(int j = 7; j >= 0; j--){
				TIM1->CNT = 0;
        while(TIM1->CNT < 60){
          if((GPIOB->IDR & (1<<5)) && (TIM1->CNT > 40)){
            break;
          }
        }

        TIM1->CNT = 0;
        while(TIM1->CNT < 80){
          if(!(GPIOB->IDR & (1<<5))){
            break;
          }
        }
        if(TIM1->CNT > 50){
          data[i] |= (1<<j);
        }
        else
          data[i] &= ~(1<<j);
			}
		}
	}

	*temp = ((float)(data[2]&0x7F)*256 + (float)data[3])/10.0;
	*hum = ((float)data[0]*256 + (float)data[1])/10.0;

}

void warning(uint16_t *ADC_ScanVal1, float temp, float hum, int *check){
	if(ADC_ScanVal1[0] > 1500  || ADC_ScanVal1[1] < 200 || ADC_ScanVal1[2] < 200 || ADC_ScanVal1[3] > 1500){
		*check = 1;
		GPIOB->BSRR |= GPIO_PIN_14|GPIO_PIN_12;
		printf("%d,%.2f,%.2f\n", 1, temp, hum); // @suppress("Float formatting support")
	}
	else{
		GPIOB->BRR |= GPIO_PIN_14;
		printf("%d,%.2f,%.2f\n", 0, temp, hum); // @suppress("Float formatting support")
	}
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

int main(void)
{
	float temp=0, hum=0;
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  HAL_TIM_Base_Start(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ScanVal, 4);
  GPIOB->BSRR |= GPIO_PIN_5;

  JUMP:
  checkFire = 0;
  while (1)
  {
	  on_voice();
	  DHT22_readData(&temp, &hum);
	  warning((uint16_t*)ADC_ScanVal, temp, hum, &checkFire);

	  UART_sendFloat(temp);
	  UART_sendData("\n");
	  Delay_ms(1000);

	  if(checkFire == 1)
		  x++;
	  if(x > 10 && ADC_ScanVal[0] < 1500 &&  ADC_ScanVal[1] > 200 && ADC_ScanVal[2] > 200 && ADC_ScanVal[3] < 1500 ){
		  GPIOB->BRR |= GPIO_PIN_12;
		  x=0;
		  goto JUMP;
	  }
  }
}

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

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Common config */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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

/* Enable DMA controller clock */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

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

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line){
}
#endif /* USE_FULL_ASSERT */
