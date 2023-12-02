/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "task.h"
#include "oled.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define OFF 0
#define ON 1
#define EnableInterrupts     __enable_irq()
#define DisableInterrupts    __disable_irq()
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t j=0;

uint16_t Out_I=0,OP_I=0;
uint16_t DIS_Out_I1=0,DIS_Out_I2=0;

uint16_t Out_V=0,OP_V=0;
uint16_t DIS_Out_V1=0,DIS_Out_V2=0;

float AD_1=0,AD1=0;
float AD_2=0,AD2=0;
float V_VOUT=0;
float V_VP=0;

float ADI_1=0,ADI1=0;
float ADI_2=0,ADI2=0;
float V_AMPL=0;
float V_AP=0;

uint16_t DAC_Value=500;
uint16_t AD_Statue=ON;
//uint16_t AD_Value[1000]={0};


uint16_t AD_Value_U[1000]={0};
uint16_t AD_Value_I[1000]={0};

uint16_t AD_U[1000]={0};
uint16_t AD_I[1000]={0};

float VPout[1000]={0};
float IPout[1000]={0};

uint32_t adcbuf1[100][2]={0};
float LX=0,LX_m=0;
float LXs=0;
float XL=0;
float a=0,b=0,c=0,d=0;

uint16_t DAC_Z[51]={
		2000,2226,2448,2663,2867,3058,3232,3387,3520,3629,3712,3768,3796,3796,3768,3712,3629,3520,3387,3232,3058,2867,2663,2448,2226,2000,1774,1552,1337,1133, 942, 768, 613, 480, 371, 288, 232, 204, 204, 232, 288, 371, 480, 613, 768, 942,1133,1337,1552,1774,2000,
		//2000,2174,2346,2515,2679,2837,2986,3127,3257,3376,3481,3573,3650,3712,3758,3787,3799,3795,3774,3737,3683,3614,3529,3430,3318,3194,3058,2913,2759,2597,2431,2260,2087,1913,1740,1569,1403,1241,1087, 942, 806, 682, 570, 471, 386, 317, 263, 226, 205, 201, 213, 242, 288, 350, 427, 519, 624, 743, 873,1014,1163,1321,1485,1654,1826,2000,
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t tmp=0,tmp1=0;
	float VPout_Value=0;
	float IPout_Value=0;

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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  Display_int();

 // HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
 // HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)DAC_Z,50,DAC_ALIGN_12B_R);
 // HAL_TIM_Base_Start(&htim1);
  // HAL_TIM_Base_Start_IT(&htim6);
 //  HAL_TIM_Base_Start(&htim15);
   HAL_TIM_Base_Start(&htim3);
   //HAL_TIM_Base_Start_IT(&htim15);
   //HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)adcbuf1,200);//启动同步规则模式及DMA�??????????????????????????3个ADC，每次采�??????????????????????????100个点*3
   HAL_Delay(100);
   HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);//ADC1
   HAL_Delay(100);
   HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);//ADC1
   HAL_Delay(200);

   /*
   HAL_ADCEx_InjectedStart_IT(&hadc2);//启动ADC中断
   HAL_ADCEx_InjectedStart(&hadc2);//启动ADC
   HAL_ADC_Start(&hadc2);

 //HAL_ADC_Start_DMA(&hadc2,(uint32_t*)AD_Value_M,100);//�??????????????????????????????????????启ADC1 DMA1
 //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Value);

//  HAL_OPAMP_Start(&hopamp1);//运放1
//  HAL_OPAMP_Start(&hopamp2);//运放2

  HAL_ADCEx_InjectedStart_IT(&hadc1);//启动ADC中断
  HAL_ADCEx_InjectedStart(&hadc1);//启动ADC
  HAL_ADC_Start(&hadc1);
*/
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start(&hadc2);
  HAL_Delay(100);
 //
 // HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)DAC_Z,50,DAC_ALIGN_12B_R);
  HAL_Delay(10);
 // HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);


  //HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)adcbuf1,200);//启动同步规则模式及DMA�??????????????????????????3个ADC，每次采�??????????????????????????100个点*3

  HAL_GPIO_WritePin(GPIOC, R0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, R1_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  TaskRemarks();//任务扫描
	 // HAL_GPIO_TogglePin(GPIOC, LED1_Pin);

	//  TaskProcess();
	 // VPout_Value=0;
	 // V_AMPL=VPout_Value;
	//  TaskProcess();
	//  HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);

	  if(AD_Statue==OFF)
	  {

		 // HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
		  TaskProcess();
         //  HAL_TIM_Base_Stop(&htim1);
           HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        //   HAL_Delay(100);
           HAL_ADC_Stop(&hadc1);
           HAL_ADC_Stop(&hadc2);

////////////////////电压计算//////////////////////
		  for(tmp=10;tmp<70;tmp++)
		  {
			  DIS_Out_V1=AD_Value_U[tmp];
			  DIS_Out_V2=AD_Value_U[tmp+1];

			  AD_1=DIS_Out_V1*3.3/4095;
			  AD_2=DIS_Out_V2*3.3/4095;

			  AD1=(AD_1-1.5);
			  AD2=(AD_2-1.5);

			  VPout[tmp-10]=sqrt(AD1*AD1+AD2*AD2);
			  VPout_Value=0;
		  }

		  for(tmp=0;tmp<30;tmp++)
		  {
			  VPout_Value=VPout_Value+VPout[tmp];
		  }

		  V_VOUT=VPout_Value/30;

////////////////////////电流计算///////////////////////////////
		  for(tmp1=10;tmp1<70;tmp1++)
		  {
			  DIS_Out_I1=AD_Value_I[tmp1];
			  DIS_Out_I2=AD_Value_I[tmp1+1];

			  ADI_1=DIS_Out_I1*3.3/4095;
			  ADI_2=DIS_Out_I2*3.3/4095;

			  ADI1=(ADI_1-1.5);
			  ADI2=(ADI_2-1.5);

			  IPout[tmp1-10]=sqrt(ADI1*ADI1+ADI2*ADI2);
			  IPout_Value=0;
		  }

		  for(tmp1=0;tmp1<30;tmp1++)
		  {
			  IPout_Value=IPout_Value+IPout[tmp1];
		  }
		  V_AMPL=IPout_Value*2/30;

		  XL=V_VOUT*10/V_AMPL/(2*3.14);




		  ////////////////////自由轴法计算//////////////////////
		  LX=0;
		  		  for(tmp=100;tmp<600;tmp++)//100 600
		  		  {

		  			//  a=AD_Value_U[tmp]*3.3/4095-1.5;
		  		//	  b=AD_Value_U[tmp+1]*3.3/4095-1.5;
		  			//  c=(AD_Value_I[tmp]*3.3/4095-1.5)*2;
		  		//	  d=(AD_Value_I[tmp+1]*3.3/4095-1.5)*2;

		  			  a=(AD_Value_U[tmp]-1900);
		  			  b=(AD_Value_U[tmp+1]-1900);
		  			  c=(AD_Value_I[tmp]-1900)*2;
		  			  d=(AD_Value_I[tmp+1]-1900)*2;


		  		//	  a=(adcbuf1[tmp][1]-1900);
		  		//	  b=(adcbuf1[tmp+1][1]-1900);
		  		//	  c=(adcbuf1[tmp][0]-1900)*2;
		  		//	  d=(adcbuf1[tmp+1][0]-1900)*2;


		  			LX_m=(b*c-a*d)/(c*c+d*d);
		  			if(LX_m<0)
		  			{LX_m=-LX_m;}

		  			  LX=LX+LX_m;
		  			//tmp++;
		  		  }
		  LXs=LX*100*2/500/120;
/*
		  a=AD_Value_U[10]*3.3/4095-1.5;
		  b=AD_Value_U[11]*3.3/4095-1.5;
		  c=(AD_Value_I[10]*3.3/4095-1.5);
		  d=(AD_Value_I[11]*3.3/4095-1.5);

		  LX=(b*c-a*d)/(c*c+d*d);

		  if(LX<0)
		  {LX=-LX;}
		  LXs=LX;
*/

///////////////////////////////////从新�?????????????中断ADC//////////////////////////////////////////
		  EnableInterrupts;
          HAL_ADC_Start(&hadc1);
          HAL_ADC_Start(&hadc2);
          HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)adcbuf1,200);
       //   HAL_TIM_Base_Start(&htim1);//重新启动采样



         // HAL_Delay(100);


	  AD_Statue=ON;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T3_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_REMAPTRIGGER_ENABLE(HAL_REMAPTRIGGER_DAC1_TRIG);
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.Timing = 0x00101D2D;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 144-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R0_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : R1_Pin R0_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R0_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void TaskDispStatus(void)
{
	Display_SET_scan(V_VOUT*1000,V_AMPL*1000);//Set_Out_I*2/10
	Display_scan(XL*1000,LXs*1000);//DIS_Out_I1
	//OLED_clear();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

	//static uint16_t i=0;

	if(htim->Instance == TIM6)//用于扫描系统函数
	{
		//HAL_TIM_Base_Stop_IT(&htim6);
		//TaskRemarks();//任务扫描
		//HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
	//	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);

		//HAL_TIM_Base_Start_IT(&htim6);

	}

	if(htim->Instance == TIM15)//用于DAC输出更新
	{

		//HAL_GPIO_TogglePin(GPIOC, LED1_Pin);

	//	  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
		//  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Value);//改变DAC�????????????????????????????????????????????????????????????????????????????????
		 // HAL_Delay(500);
		//  DAC_Value=DAC_Z[i++];
	//	  if(i>=65)
	//	  {
	//		  i=0;
	//	  }
	}


    //if (htim->Instance == htim14.Instance)
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint16_t i=0,j=0;

	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);

	//if(hadc == &hadc1)
	{AD_Value_I[i]=HAL_ADC_GetValue(&hadc1);i++;}
	//if(hadc == &hadc2)
	{AD_Value_U[j]=HAL_ADC_GetValue(&hadc2);j++;}

	if(j>1000)
	{
		i=0;
		j=0;
		DisableInterrupts;
		AD_Statue=OFF;
		//HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
	}
}

/*
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//HAL_ADCEx_InjectedStop_IT(&hadc1);//关闭ADC中断
	static uint16_t i=0,j=0;

	//AD_Statue=OFF;


	if(hadc == &hadc1)
		{

		AD_Value_I[i] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	//	OP_I=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	//	AD_Value_U[i]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
		//AD_Value[i]=Out_I;
		i++;
		HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
		if(i>1000)
		{
		//	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
			i=0;
		//	AD_Statue=OFF;
		//	DisableInterrupts;
		}
		}

	if((hadc == &hadc2)&&(AD_Statue==ON))
		{

		AD_Value_U[j] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	//	OP_I=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	//	AD_Value_U[i]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
		j++;
		HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
		if(j>1000)
		{
		//	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
			j=0;
			AD_Statue=OFF;
			DisableInterrupts;
		}
		}

}
*/

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
