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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "oled.h"
#include "encoder.h"
#include "motor.h"
#include "vehicleservo.h"
#include "diff.h"
#include "pid.h"
#include "speed.h"
#include "dist.h"
#include "uart.h"
#include "icm20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// read-only data (data that are taken from sensors)
typedef struct {
	encoder_control *motorLencoder;
	encoder_control *motorRencoder;
}Encoder_Data;

typedef struct {
	pid_control *motorLpid;
	pid_control *motorRpid;
	pid_control *anglepid;
}PID_Data;

typedef struct{
	motor_control *motorL;
	motor_control *motorR;
}Motor_Data;

typedef struct{
	vehicleservo_control *vehicleservo;
}VehicleServo_Data;

typedef struct{
	speed_subcontrol* motorLspeed;
	speed_subcontrol* motorRspeed;
	angle_subcontrol* servoangle;
	speed_control* mainspeed;
}Speed_Data;

typedef struct{
	float pid_kp_l;
	float pid_ki_l;
	float pid_kp_r;
	float pid_ki_r;
	float pid_kp_a;
	float pid_ki_a;
}Diff_Data;


// write-only data (data that are used mostly for controlling output Pheripheral )
typedef struct{
	uint8_t aRxBuffer[10];
    uint8_t aTxBuffer[10];
}UART_Data;


typedef struct{
	int stage;
	double distance;
	double distToObsEndtoEnd;
	double distToObsEdge;
	double distToObs;
	float ultrasensordist;
	int IRdistR;
	int IRdistL;
}Task2;


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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PathBuilderTask */
osThreadId_t PathBuilderTaskHandle;
const osThreadAttr_t PathBuilderTask_attributes = {
  .name = "PathBuilderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

Encoder_Data encoderData;
PID_Data pidData;
Motor_Data motorData;
Diff_Data diffData;
UART_Data uartData;

Speed_Data speedData;
VehicleServo_Data vehicleservoData;

Task2 task2data;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void StartUartTask(void *argument);
void StartShowTask(void *argument);
void StartPathBuilderTask(void *argument);
void IR(void *argument);

/* USER CODE BEGIN PFP */
void uart_switch(uint8_t* buff);
void run_movement_command(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



int isnumchar(uint8_t cell){
	if (cell < 0x3a && cell > 0x29) return 1;
	else return 0;
}

int isalphachar(uint8_t cell){
	if (cell < 0x5b && cell > 0x40) return 1;
	else if (cell < 0x7b && cell > 0x60) return 1;
	else return 0;
}

void uart_switch(uint8_t * buff){
	  char aRead1[4];
	  char aRead2[4];

	  if 		(isalphachar(buff[0])
			  && isalphachar(buff[5])) {


		  sprintf(aRead1,"%c%c%c%c", (uartData.aRxBuffer[1]),(uartData.aRxBuffer[2]),(uartData.aRxBuffer[3]),(uartData.aRxBuffer[4]));
		  int n = atoi(aRead1);  //now n is 49 50 48 48

		  sprintf(aRead2,"%c%c%c%c", (uartData.aRxBuffer[6]),(uartData.aRxBuffer[7]),(uartData.aRxBuffer[8]),(uartData.aRxBuffer[9]));
		  int m = atoi(aRead2);  //now n is 49 50 48 48


		 switch(buff[0]){

			  case UART_FORWARD:
					//n = dist_get_motor_diststeps((double) n,0);
				  //icm20948_init(&hi2c1);
				speed_control_path_setup(speedData.mainspeed, 20, 100, (double) n, ((float)m) / 100,0,0,PATH_MODE_STR);
				speed_control_path_start(speedData.mainspeed);
				break;
			  case UART_FORWARD_RIGHT:
				//n = dist_get_motor_diststeps((double) n,0);
				  //icm20948_init(&hi2c1);
				speed_control_path_setup(speedData.mainspeed, 20, 100, (double) n, ((float)m) / 100,90,10,PATH_MODE_CURVE);
				speed_control_path_start(speedData.mainspeed);
				break;

			  case UART_FORWARD_LEFT:
				//n = dist_get_motor_diststeps((double) n,0);
				  //icm20948_init(&hi2c1);
				speed_control_path_setup(speedData.mainspeed, 20, 100, (double) n, -((float)m) / 100,90,10,PATH_MODE_CURVE);
				speed_control_path_start(speedData.mainspeed);
				break;

			  case UART_REVERSE:
				  //icm20948_init(&hi2c1);
				speed_control_path_setup(speedData.mainspeed, -20, 100, (double) n, ((float)m) / 100,0,0,PATH_MODE_STR);
				speed_control_path_start(speedData.mainspeed);
				break;

			  case UART_REVERSE_RIGHT:
				 // icm20948_init(&hi2c1);
				//n = dist_get_motor_diststeps((double) n,0);
				speed_control_path_setup(speedData.mainspeed, -20, 100, (double) n, ((float)m) / 100,90,10,PATH_MODE_CURVE);
				speed_control_path_start(speedData.mainspeed);
				break;

			  case UART_REVERSE_LEFT:
				 // icm20948_init(&hi2c1);
				//n = dist_get_motor_diststeps((double) n,0);
				speed_control_path_setup(speedData.mainspeed, -20, 100, (double) n, -((float)m) / 100,90,10,PATH_MODE_CURVE);
				speed_control_path_start(speedData.mainspeed);
				break;



			  case UART_GYRO_BIAS:
			    angle_subcontrol_set_leftanglemul(speedData.servoangle, (float) n / (float) 100 );
			    angle_subcontrol_set_rightanglemul(speedData.servoangle, (float) m / (float) 100);
				HAL_UART_Transmit(&huart3, (uint8_t *) "0000000000", 10, 0xFFFF);
				break;

			  case UART_AUTOCORRECT_TOGGLE:

				speed_control_toggle_correction(speedData.mainspeed,(float) n);
				HAL_UART_Transmit(&huart3, (uint8_t *) "0000000000", 10, 0xFFFF);
				break;
			  case UART_PROPORTION:

				diffData.pid_kp_l = (float) n / (float) 100;
				diffData.pid_kp_r = (float) m / (float) 100;
				HAL_UART_Transmit(&huart3, (uint8_t *) "0000000000", 10, 0xFFFF);
				break;

			  case UART_INTEGRAL:

				diffData.pid_ki_l = (float) n / (float) 10000;
				diffData.pid_ki_r = (float) m / (float) 10000;
				HAL_UART_Transmit(&huart3, (uint8_t *) "0000000000", 10, 0xFFFF);

				break;

			  case UART_START_TASK2:
				  task2data.stage = 0;

				  break;
			  case UART_POLL:
				  task2data.ultrasensordist	= m;
				  break;

			  default:
				HAL_UART_Transmit(&huart3, (uint8_t *) "FFFFFFFFFF", 10, 0xFFFF);

			}

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
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3,  (uint8_t *) uartData.aRxBuffer, 10);
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

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(StartUartTask, NULL, &UART_Task_attributes);

  /* creation of ShowTask */
  ShowTaskHandle = osThreadNew(StartShowTask, NULL, &ShowTask_attributes);

  /* creation of PathBuilderTask */
  PathBuilderTaskHandle = osThreadNew(StartPathBuilderTask, NULL, &PathBuilderTask_attributes);

  /* creation of IRTask */
  IRTaskHandle = osThreadNew(IR, NULL, &IRTask_attributes);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
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
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16 -1 ;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           PE10 PE12 */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t Value1, Value2, Time, State, Distance = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim1)											   // Input Capture Mode
{
		if (State == 0)
		{
			Value1 = HAL_TIM_ReadCapturedValue(htim1, TIM_CHANNEL_4); 								   // Save First Value at Value1 ( RISING EDGE )
			State = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);	   // Change Capture Polarity to FALLING EDGE
		}

		else if (State == 1)
		{
			Value2 = HAL_TIM_ReadCapturedValue(htim1, TIM_CHANNEL_4);  								   // Save Second Value at Value2 ( FALLING EDGE )

			if (Value2 > Value1)
			{
				Time = Value2-Value1;
			}

			else if (Value1 > Value2)
			{
				Time = (65536 - Value1) + Value2;
			}

			Distance = Time * 0.034 / 2;
			State = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);	   // Change Capture Polarity to RISING EDGE
		}
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

int distance = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit(&huart3, (uint8_t *) uartData.aRxBuffer, 10, 0xFFFF);

//	uint8_t test[20];
//	sprintf(test, "%s\0", uartData.aRxBuffer);
//	OLED_ShowString(10,10,test,20);
	//OLED_Refresh_Gram();

	UNUSED(huart);
	uart_switch(uartData.aRxBuffer);
	HAL_UART_Receive_IT(&huart3, (uint8_t *) uartData.aRxBuffer,10);
 }

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
//	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)){
//
//	  }
	  //speed_control_run(speedData.mainspeed);
	  osDelay(6000000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* Infinite loop */


  for(;;){
	  //HAL_UART_Transmit(&huart3, (uint8_t*) "I2566x2566", 10, 0xFFFF);
	  //HAL_UART_Receive (&huart3, receive, 4, 0xFFFF);  // receive 4 bytes of data

	  osDelay(100);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartShowTask */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShowTask */
void StartShowTask(void *argument)
{
  /* USER CODE BEGIN StartShowTask */
  /* Infinite loop */
	 OLED_Init();
	 int arr[20];
	 OLED_Clear();

	 GPIO_PinState prevstate;
	 GPIO_PinState currstate = GPIO_PIN_RESET;
	 int screenstate = 3;
	 int totalstate = 3;


  for(;;)
  {

	prevstate = currstate;
	currstate = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);

	if (prevstate == GPIO_PIN_RESET  && currstate == GPIO_PIN_SET){
		 if (screenstate < totalstate - 1) screenstate++;
		 else screenstate = 0;
		 OLED_Clear();
		 OLED_Refresh_Gram();
	 }
	//int distance = 100;
	prevstate = currstate;
	 float tickprogl = speed_control_get_tickprogress_L(speedData.mainspeed)*100;
	 float tickprogr = speed_control_get_tickprogress_R(speedData.mainspeed)*100;
	 float pulseprogl = speed_control_get_pulseprogress_L(speedData.mainspeed)*100;
	 float pulseprogr = speed_control_get_pulseprogress_R(speedData.mainspeed)*100;
	 switch(screenstate){

		 case 0:
			 sprintf(arr, "Tick Based");
			 OLED_ShowString(10,0,arr,20);
			 sprintf(arr, "Lprogp :%6d", (int)round(tickprogl));
			 OLED_ShowString(10,10,arr,20);
			 sprintf(arr, "Rprogp :%6d", (int)round(tickprogr));
			 OLED_ShowString(10,20,arr,20);
			 sprintf(arr, "reqdis :%6d", (int)round(speedData.mainspeed->neededTotalTick));
			 OLED_ShowString(10,30,arr,20);
			 sprintf(arr, "Lprgdis:%6d",(int)round(speedData.mainspeed->LmotorProgressTick));
			 OLED_ShowString(10,40,arr,20);
			 sprintf(arr, "Rprodis:%6d", (int)round(speedData.mainspeed->RmotorProgressTick));
			 OLED_ShowString(10,50,arr,20);
		 break;
		 case 1:

			 sprintf(arr, "Pulse Based");
			 OLED_ShowString(10,0,arr,20);
			 sprintf(arr, "Lprogp :%6d", (int)round(pulseprogl));
			 OLED_ShowString(10,10,arr,20);
			 sprintf(arr, "Rprogp :%6d", (int)round(pulseprogr));
			 OLED_ShowString(10,20,arr,20);
			 sprintf(arr, "reqdis :%6d", (int)round(speedData.mainspeed->neededTotalPulse));
			 OLED_ShowString(10,30,arr,20);
			 sprintf(arr, "Lprgdis:%6d",(int)round(speedData.mainspeed->LmotorProgressPulse));
			 OLED_ShowString(10,40,arr,20);
			 sprintf(arr, "Rprodis:%6d", (int)round(speedData.mainspeed->RmotorProgressPulse));
			 OLED_ShowString(10,50,arr,20);
		 break;
		 case 2:
			 sprintf(arr, "Speed & Error");
			 OLED_ShowString(10,0,arr,20);
			 sprintf(arr, "LMotor :%6d", (int)round(speedData.motorLspeed->speed));
			 OLED_ShowString(10,10,arr,20);
			 sprintf(arr, "Rmotor :%6d", (int)round(speedData.motorRspeed->speed));
			 OLED_ShowString(10,20,arr,20);
			 sprintf(arr, "WA     :%6d", (int)round(speedData.mainspeed->requestedAngle));
			 OLED_ShowString(10,30,arr,20);
			 sprintf(arr, "Lerr   :%6d", (int)round(pidData.motorLpid->curr->Error));
			 OLED_ShowString(10,40,arr,20);
			 sprintf(arr, "Rerr   :%6d",(int)round(pidData.motorRpid->curr->Error));
			 OLED_ShowString(10,50,arr,20);
       break;

   }
	 OLED_Refresh_Gram();
	 osDelay(500); //every half a second
  }
  /* USER CODE END StartShowTask */
}

/* USER CODE BEGIN Header_StartPathBuilderTask */
/**
* @brief Function implementing the PathBuilderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPathBuilderTask */
void StartPathBuilderTask(void *argument)
{
  /* USER CODE BEGIN StartPathBuilderTask */


	// VehicleServo Control
	vehicleservoData.vehicleservo = vehicleservo_init(&htim1, TIM_CHANNEL_1);

	//Motor Control
	motor_control_pin pin2, pin3, pin4, pin5;

	pin2.GPIOx = GPIOA; pin2.GPIO_Pin = GPIO_PIN_2;
	pin3.GPIOx = GPIOA; pin3.GPIO_Pin = GPIO_PIN_3;
	pin4.GPIOx = GPIOC; pin4.GPIO_Pin = GPIO_PIN_5;
	pin5.GPIOx = GPIOE; pin5.GPIO_Pin = GPIO_PIN_12;

	motorData.motorL = motor_init(&htim8, TIM_CHANNEL_1, pin3, pin2);
	motorData.motorR = motor_init(&htim8, TIM_CHANNEL_3, pin5, pin4);

	//Encoder Control
	encoderData.motorLencoder = encoder_init(&htim2);
	encoderData.motorRencoder = encoder_init(&htim4);

	//PID_Control
	//set default
	diffData.pid_ki_l = 0.01;
	diffData.pid_kp_l = 12;
	diffData.pid_ki_r = 0.01;
	diffData.pid_kp_r = 11;
	diffData.pid_ki_a = 0.0005;
	diffData.pid_kp_a = 8;

	pidData.motorLpid = pid_init(&(diffData.pid_kp_l),&(diffData.pid_ki_l)); //hyper parameter
	pidData.motorRpid = pid_init(&(diffData.pid_kp_r),&(diffData.pid_ki_r)); // hyper parameter
	pidData.anglepid = pid_init(&(diffData.pid_kp_a),&(diffData.pid_ki_a)); //hyper parameter

	//Speed_Control
	speedData.motorLspeed = speed_subcontrol_init(pidData.motorLpid,motorData.motorL,encoderData.motorLencoder);
	speedData.motorRspeed = speed_subcontrol_init(pidData.motorRpid,motorData.motorR,encoderData.motorRencoder);


	//gyro control
	icm20948_init(&hi2c1);
	speedData.servoangle = angle_subcontrol_init(pidData.anglepid,vehicleservoData.vehicleservo,&hi2c1);

	//Main Speed_Control
	speedData.mainspeed = speed_control_init(speedData.motorLspeed,speedData.motorRspeed,speedData.servoangle,&huart3);

	speed_control_set_callbackdistptr(speedData.mainspeed, &(task2data.distance));

  task2data.distToObs = 0;
  task2data.distToObsEdge = 0;
  task2data.ultrasensordist = 999;

  task2data.distance = 0;
  task2data.stage = -1;


  double Ladd = 0, Radd =0;

  double Lencodersum = 0;
  double Rencodersum = 0;
  double idealencodersum =0;

  vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);



  /* Infinite loop */


  for(;;)
  {


	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)){
		 speed_control_run(speedData.mainspeed);
	 }else{

	 //fastest path tasks uses lower level pwm control over pid due to accuary issues

	 encoder_run(speedData.mainspeed->Lmotor_speed_subctl->encoder_ctl,200,ENCODER_DIRECTION_NATURAL);
	 encoder_run(speedData.mainspeed->Rmotor_speed_subctl->encoder_ctl,200,ENCODER_DIRECTION_FLIPPED);
	 Ladd = speedData.mainspeed->Lmotor_speed_subctl->encoder_ctl->currFrame->pulseDiff;
	 Radd = speedData.mainspeed->Rmotor_speed_subctl->encoder_ctl->currFrame->pulseDiff;
	 Lencodersum += Ladd;
	 Rencodersum += Radd;
	 idealencodersum = Lencodersum + Rencodersum;

	  switch(task2data.stage){
	  	  case 0:
	  		 vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 3980),MOTOR_PWM_NORMAL);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl, diff_get_Rmotor_values(0, 4000),MOTOR_PWM_NORMAL);
			 if(task2data.ultrasensordist < 50) {
				 task2data.stage = 1;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }else if ( ((double)task2data.IRdistL /(double) task2data.IRdistR) > 1.23 || ((double)task2data.IRdistR / (double)task2data.IRdistL) > 1.23  ){
				 task2data.stage = 18;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 1: //brake
		  		 vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, -30);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(-30, 0),MOTOR_PWM_BRAKE);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(-30, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 2;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 2: //turn 90 left
	  		vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, -30);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(-30, 4950),MOTOR_PWM_NORMAL);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(-30, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 250 /* turn distance*/) {
				 task2data.stage = 3;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 3: //brake
	  		vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 4;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 4: //str parallel
	  		vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0,4950),MOTOR_PWM_NORMAL);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 40  /* str distance*/) {
				 task2data.stage = 5;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 5: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 6;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 6: // turn 180
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(30, 4950),MOTOR_PWM_NORMAL);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(30, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum >  800  /* turn distance*/) {
				 task2data.stage = 7;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 7: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 8;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 8: //str parallel
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 4950),MOTOR_PWM_NORMAL);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 1320/*distance*/) {
				 task2data.stage = 9;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 9: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 10;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 10: // turn 180
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(30, 4950),MOTOR_PWM_NORMAL);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(30, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 800 /* turn distance*/) {
				 task2data.stage = 11;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 11: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 12;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 12: //str parallel
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 4950),MOTOR_PWM_NORMAL);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 40/* turn distance*/) {
				 task2data.stage = 13;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 13: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, -30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(-30, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(-30, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 14;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 14: // turn 90
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, -30);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(-30, 5000),MOTOR_PWM_NORMAL);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(-30, 5000),MOTOR_PWM_NORMAL);
			 if (idealencodersum > 250 /* turn distance*/) {
				 task2data.stage = 15;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 15: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			 if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 16;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 16:
	  		 vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 4950),MOTOR_PWM_NORMAL);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl, diff_get_Rmotor_values(0, 5000),MOTOR_PWM_NORMAL);
			 if(task2data.ultrasensordist < 30 || (task2data.IRdistR > 2130 &&  task2data.IRdistL > 2130)) {
				 task2data.stage = 17;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;
	  	  case 17: //brake
		  	vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);


			if (Ladd == 0 && Radd == 0) {
				 task2data.stage = -1;
				 HAL_UART_Transmit(&huart3, (uint8_t*) "0000000000", 10, 0xFFFF);
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;


		  //edgecase
	  	  case 18:
			vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, 0),MOTOR_PWM_BRAKE);
			motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl,diff_get_Rmotor_values(0, 0),MOTOR_PWM_BRAKE);
			if (Ladd == 0 && Radd == 0) {
				 task2data.stage = 19;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;

	  	  case 19:
	  		 vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl, 0);
			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, diff_get_Lmotor_values(0, -3920),MOTOR_PWM_NORMAL);
			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl, diff_get_Rmotor_values(0, -4000),MOTOR_PWM_NORMAL);
			 if (idealencodersum < -100 ) {
				 task2data.stage = 1;
				 Lencodersum = 0;
				 Rencodersum = 0;
			 }
			 break;


	  }
	 }

    osDelay(100);
  }
  /* USER CODE END StartPathBuilderTask */
}

/* USER CODE BEGIN Header_IR */
/**
* @brief Function implementing the IRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IR */
void IR(void *argument)
{
  /* USER CODE BEGIN IR */
	  /* Infinite loop */
		uint16_t IR_top, IR_bottom;
		char distance_top[10], distance_bottom[10];
	  for(;;)
	  {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_Start(&hadc2);

		  IR_top = HAL_ADC_GetValue(&hadc1);
		  IR_bottom = HAL_ADC_GetValue(&hadc2);

//		  sprintf(distance_top, "%d", IR_top);
//		  sprintf(distance_bottom, "%d", IR_bottom);
//
//		  char distanceCombined[10];

		  task2data.IRdistR = ((int)IR_top*0.8) + (task2data.IRdistR*0.2) ;
		  task2data.IRdistL = ((int)IR_bottom*0.8)+( task2data.IRdistL*0.2) ;

		  //task2data.IRdist = ((int)IR_top + (int)IR_bottom + task2data.IRdist + task2data.IRdist )/4;

//		  char command = 'I';
//		  char separator = 'x';
//
//		  sprintf(distanceCombined, "%c%d%c%d", command, IR_top, separator, IR_bottom);

		  //HAL_UART_Transmit(&huart3, (uint8_t*) distanceCombined, 10, 0xFFFF);
		  //osDelay(10);
		  //HAL_UART_Transmit(&huart3, (uint8_t*) distance_bottom, 5, 0xFFFF);


	    osDelay(50);
  }
  /* USER CODE END IR */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

//  switch(task2data.stage){
//		 case 0:
//			 // start stage 1 & start movement towards obstacle
//
//			 //move vehicle
////			 speed_control_path_setup(speedData.mainspeed,speed -10,  100, 3000,0,0,0,PATH_MODE_STR);
////			 speed_control_path_start(speedData.mainspeed);
//			 //105L-152M-240R
//			 vehicleservo_set_wheel_angle(speedData.mainspeed->angle_subctl->vehicleservo_ctl,152);
//			 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, 500,MOTOR_PWM_NORMAL);
//			 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl, 500,MOTOR_PWM_NORMAL);
//
//			 //set stage 1
//			 task2data.stage = 1;
//			 break;
//
//		 case 1:
//
//			 if ( task2data.IRdist > 2000 ){
//				 //break movement
//				 //speed_control_path_break(speedData.mainspeed, &task2data.distance);
//				 //set return distance to distToObs
//				 //task2data.distToObs = task2data.distance;
//				 //set return distance to 0
//				 //task2data.distance = 0;
//				 //reverse for turning space
////				 speed_control_path_setup(speedData.mainspeed,-speed + 10, 100, 100,0,0,0,PATH_MODE_STR);
////				 speed_control_path_start(speedData.mainspeed);
//
//				 motor_set_pwm(speedData.mainspeed->Lmotor_speed_subctl->motor_ctl, -500,MOTOR_PWM_NORMAL);
//				 motor_set_pwm(speedData.mainspeed->Rmotor_speed_subctl->motor_ctl, -500,MOTOR_PWM_NORMAL);
//
//				 task2data.stage = 2;
//			 }
//
//
//			 break;
//
//		 case 2:
//		 // stop movement twd obstacle when we reach 45 cm then record distance travelled for later
//		 // start stage 3 & turn left 90 degree
//
//		 if ( task2data.distance != 0 ){
//
//			 //set return distance to 0
//			 task2data.distance = 0;
//			 //turn left 90
//			 speed_control_toggle_correction(speedData.mainspeed, 5);
//			 speed_control_path_setup(speedData.mainspeed,speed, 100, 83,-30,90,25,PATH_MODE_CURVE);
//			 speed_control_path_start(speedData.mainspeed);
//			 speed_control_toggle_correction(speedData.mainspeed, 5);
//			 //set stage 3
//			 task2data.stage = 3;
//		 }
//
//
//		 case 3:
//			 //wait for vehicle to complete 90 degree turn
//			 //start stage 4 & move foward parallel to obstacle
//
//			 if (/*path completed / distance returned*/ task2data.distance != 0){
//				 task2data.distance = 0;
//				 //move vehicle
//				 speed_control_path_setup(speedData.mainspeed, speed, 100, 3000,0,0,0,PATH_MODE_STR);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 3
//				 task2data.stage = 4;
//			 }
//			 break;
//
//		 case 4:
//			 //stop movement beside obstacle when obstacle parallel end then record distance travelled for later
//			 //start stage 5 & turn 180 right around obstacle
//
//			 if (/*IR meet condition*/task2data.ultrasensordist>30){
//				 //break movement
//				 speed_control_path_break(speedData.mainspeed, &task2data.distance);
//				 //set return distance to distToObsEdge
//				 task2data.distToObsEdge = task2data.distance;
//				 //set return distance to 0
//				 task2data.distance = 0;
//				 //turn right 180
//				 speed_control_toggle_correction(speedData.mainspeed, 5);
//				 speed_control_path_setup(speedData.mainspeed, speed, 100, 120, 30,180,30,PATH_MODE_CURVE);
//				 speed_control_path_start(speedData.mainspeed);
//				 speed_control_toggle_correction(speedData.mainspeed, 5);
//				 //set stage 4
//				 task2data.stage = 5;
//			 }
//			 break;
//		 case 5:
//			 //wait for vehicle to complete 180 degree turn
//			 //start stage 6 & move foward parallel to obstacle
//
//			 if (task2data.distance != 0){
//				 task2data.distance = 0;
//				 //move vehicle
//				 speed_control_path_setup(speedData.mainspeed, speed, 100,3000,0,0,0,PATH_MODE_STR);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 5
//				 task2data.stage = 6;
//			 }
//			 break;
//
//		 case 6:
//			 // first reading after turn
//
//			if(task2data.ultrasensordist > 30 && gap == 0) overob = 1;
//
//			if(task2data.ultrasensordist <= 30) gap = 0;
//			else gap = 1;
//
//
//
//			 //stop movement beside obstacle when obstacle parallel end then record distance travelled for later
//			 //start stage 6 & turn 180 right around obstacle
//			 if (overob == 1){
//				 //break movement
//				 speed_control_path_break(speedData.mainspeed, &task2data.distance);
//				 //set return distance to distToObs
//				 task2data.distToObsEndtoEnd = task2data.distance;
//				 //set return distance to 0
//				 task2data.distance = 0;
//				 //turn left 180
//				 speed_control_path_setup(speedData.mainspeed, speed, 100, 0, 30,180,5,PATH_MODE_CURVE);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 6
//				 task2data.stage = 7;
//
//			 }
//			 break;
//
//
//		 case 7:
//
//			 //wait for vehicle to complete 180 degree turn
//			 //start stage 7 & move fwd parallel to obstacle
//			 if (/*path completed / distance returned*/task2data.distance != 0){
//				 task2data.distance = 0;
//				 //move vehicle
//				 speed_control_path_setup(speedData.mainspeed, speed, 100,task2data.distToObsEndtoEnd - task2data.distToObsEdge ,0,0,0,PATH_MODE_STR);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 7
//				 task2data.stage = 8;
//			 }
//			 break;
//
//		 case 8:
//
//			 //wait for vehicle to complete calculated 90 left turn
//			 //start stage 8 & move foward to start/end point
//			 if (/*path completed / distance returned*/task2data.distance != 0){
//				 //break movement
//				 speed_control_path_break(speedData.mainspeed, &(task2data.distance));
//				 //set return distance to distToObs
//
//				 //set return distance to 0
//				 task2data.distance = 0;
//				 //turn left 90
//				 speed_control_path_setup(speedData.mainspeed, speed, 100, 0,-30,90,5,PATH_MODE_CURVE);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 7
//				 task2data.stage = 9;
//			 }
//			 break;
//		 case 9:
//			 //wait for vehicle to complete str movement
//			 //start stage 9 to end path
//			 if (/*path completed / distance returned*/task2data.distance != 0){
//				 task2data.distance = 0;
//				 //move vehicle
//				 speed_control_path_setup(speedData.mainspeed, speed, 100,task2data.distToObsEdge ,0,0,0,PATH_MODE_STR);
//				 speed_control_path_start(speedData.mainspeed);
//				 //set stage 8
//				 task2data.stage = 10;
//			 }
//			 break;
//
//		 case 11:
//			 if(task2data.IRdist > 2000){
//				 speed_control_path_break(speedData.mainspeed, &(task2data.distance));
//				 //set return distance to 0
//				 task2data.distance = 0;
//				 HAL_UART_Transmit(&huart3, (uint8_t*) "0000000000", 10, 0xFFFF);
//			 }
//			 break;
//
//		 default:
//			 break;
//
//	 }
