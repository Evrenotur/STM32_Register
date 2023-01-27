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
#include "cmsis_os.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int encoder,encoder2,angle,angle2;
double kp = 50.20, ki = 0.0, kd = 15.37; // CGO TETA 1
double kp2 = 51.79, ki2 = 9.13, kd2 =16.63; // CGO TETA 2


double input, output, setpoint;
double iTerm = 0, lastInput = 0, dInput = 0, error = 0;
double outMin =-50, outMax = 50;
double sampleTime = 10;


double input2, output2, setpoint2;
double iTerm2 = 0, lastInput2 = 0, dInput2 = 0, error2 = 0;

double teta1=0;
double teta2=0;
float c,tim,tim2;
int u;

float x;
float y;
float L1=17.75;
float  L2=8.25;

double o,p;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}






void pwmOut(int out) {                                // to H-Bridge board

  if (out >0) {
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,out);
	    	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
  }
  else {

	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);

	    	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,out);


  }
}

void pwmOut2(int out) {                                // to H-Bridge board

  if (out > 0) {
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,out);
	    	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
  }

  else {

	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);

	    	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,out);


  }
}


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 26;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	// encoder= __HAL_TIM_GET_COUNTER(&htim3);
	 //encoder2= __HAL_TIM_GET_COUNTER(&htim4);

	  if (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)) {
			      HAL_Delay(30);
			          c +=15;
			      while(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)){
			      }
			      HAL_Delay(10);

			      }


			  if (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)) {
			      HAL_Delay(30);
			 	          c -=15;
			 	      while(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)){
			 	      }
			 	      HAL_Delay(10);

			 	      }

 // __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,50);
  //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	// __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,75);
	// __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,90);
	 // angle=  MAP(encoder,0,65535,0,360);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	//  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,50);
	 // encoder= __HAL_TIM_GET_COUNTER(&htim3);



    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {

	  tim++;
	 // u+=tim;

      if(tim<200)
      {


            x=3;
            y=16;


    	 teta2= atan2(sqrt(1-((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))*((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))),
    	  	 			             	          			        ((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)));

    	teta1=atan2(y,x) - atan2(sqrt(y*y + x*x - pow((L2*cos(teta2)+L1),2)), (L2*cos(teta2)+L1));

    	teta2+=teta2*57.29;
    	teta1+=teta1*57.29;
    	  //	 teta2 +=113;
    	  	// teta1 +=51;

      }


	  	   if(tim>250&&tim<300)
	  	  {
	  		 // x=5;
	  		y=16;
	  				  for(x=3;x<5;x+=0.1){

	  		//int o=123;
	  	//int	p=42;

	  		o = atan2(sqrt(1-((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))*((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))),
	  		             	          			          ((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)));

	  	p=atan2(y,x) - atan2(sqrt(y*y + x*x - pow((L2*cos(o)+L1),2)), (L2*cos(o)+L1));

	  	o=o*57.29;
		p=p*57.29;
	  				  }
	  	if(teta2>o)
	  	{
	  teta2-=teta2-o;
	  }
	  	else
	 {
	 teta2 +=o-teta2;
	 }

	  if(teta1>p)
	  {
	  teta1 -=teta1-p;
	  }
	  else
	  {
	  	teta1 +=p-teta1;
	 }
	   }

	   else if(tim>400&&tim<600)
	  		{

	  		  x=5;
	  		 // y=14;

	    	//int	o=127;
	    		//int p=50;

	  		  for(y=16;y>14;y-=0.1){
	  	  o = atan2(sqrt(1-((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))*((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))),
	  	  			             	          			            ((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)));

	  	 	p=atan2(y,x) - atan2(sqrt(y*y + x*x - pow((L2*cos(o)+L1),2)), (L2*cos(o)+L1));

	  		o=o*57.29;
	  		p=p*57.29;
	  		  }
	  	  		if(teta2>o)
	  	  		{
	  	  			teta2-=teta2-o;
	  	  		}
	  	  		else
	  	  		{
	  	  			teta2 +=o-teta2;
	  	  		}

	  	  		if(teta1>p)
	  	  		{
	  	  			teta1 -=teta1-p;
	  	  		}
	  	  		else
	  	  		{
	  	  			teta1 +=p-teta1;
	  	  		}
	  		  	  }
/*
	  	  else if(tim>600&&tim<800)
	  	  {
	  		  x=3;
	  		  //y=16;

	  		//int  o=113;
	  		//int p=51;

	  		  for(y=14;y<16;y+=0.1){
	  		 o= atan2(sqrt(1-((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))*((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2))),
	  			             	          			           ((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)));
	  		  p=atan2(y,x) - atan2(sqrt(y*y + x*x - pow((L2*cos(o)+L1),2)), (L2*cos(o)+L1));

	  		o=o*57.29;
	  	  	p=p*57.29;
	  		  }
	  		if(teta2>o)

	  		{
	  			  teta2-=teta2-o;
	  		}
	  		else
	  	   {
	  	    teta2 +=o-teta2;
	  	    }
	  	   if(teta1>p)
	  	   {
	  	  teta1 -=teta1-p;
	  		}
	  		else
	  		{
	  	  teta1 +=p-teta1;
	  	    }

	  	  }
*/
		 encoder2= __HAL_TIM_GET_COUNTER(&htim4);

	  int value = MAP(teta2,0,360,0,30000);
	 	  setpoint2 = value;                                    // set position
	 	  input2 =   encoder2 ;                                 // data from encoder
	 	  error2 = setpoint2 - input2;
	 	  iTerm2 += ki2 * error2 * sampleTime;
	 	  if (iTerm2 > outMax) iTerm2= outMax;                  // prevent iTerm windup
	 	  else if (iTerm2 < outMin) iTerm2 = outMin;
	 	  dInput2 = (input2 - lastInput2) / sampleTime;
	 	  output2 = kp2 * error2 + iTerm2 - kd2 * dInput2;          // compute PID Output
	 	  if (output2 > outMax) output2 = outMax;               // limit output
	 	  else if (output2 < outMin) output2 = outMin;
	 	  lastInput2 = input2;
	 	 pwmOut2(output2);
		  angle2=  MAP(encoder2,0,65535,0,360);

		  angle2 = (angle2*2)+8;


		  encoder= __HAL_TIM_GET_COUNTER(&htim3);

		  int value2 = MAP(teta1,0,360,0,30000);
		 	  setpoint = value2;           // set position
		 	  input =   encoder ;                                 // data from encoder
		 	  error = setpoint - input;
		 	  iTerm += ki * error * sampleTime;
		 	  if (iTerm > outMax) iTerm= outMax;                 // prevent iTerm windup
		 	  else if (iTerm < outMin) iTerm = outMin;
		 	  dInput = (input - lastInput) / sampleTime;
		 	  output = kp * error + iTerm - kd * dInput;          // compute PID Output
		 	  if (output > outMax) output = outMax;               // limit output
		 	  else if (output < outMin) output = outMin;
		 	  lastInput = input;
			 pwmOut(output);
			  angle=  MAP(encoder,0,65535,0,360);

			  angle = (angle*2)+8;



		// __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);



    osDelay(10);
  }
  /* USER CODE END StartTask03 */
}

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
