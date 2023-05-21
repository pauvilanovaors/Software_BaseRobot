/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "timers.h"
#include <stdio.h>
#include <string.h>
#include "timers.h"
#include "tof.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	STOP_STATE = 0,
	//IDLE_STATE,
	MOVING_STATE,
//	LIFTER_STATE,
	PACKET_STATE
}dev_state_t;

typedef enum{
	UP_DIR = 0,
	DOWN_DIR,
	STOP_DIR
}lifter_motor_direction_t;

typedef enum{
	FULL_STEP = 0,
	HALF_STEP,
	MICROSTEPPING_1_4,
	MICROSTEPPING_8,
	MICROSTEPPING_16,
	MICROSTEPPING_32
}microstepping_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IS_VAL_INRANGE(value, max_value, min_value)((value <= max_value) && (value >= min_value))

#define CALC_X_VALUE(x_num)(x_num - 100)
#define CALC_Y_VALUE(y_num)(y_num - 201)
#define CALC_SPEED(speed)(speed-9)

#define CALC_VEL(var,speed)((10*speed)*var*(2./50)*(1000./36000))

#define SET_DEVICE_STATE(state)(dev_state = state)
#define GET_DEVICE_STATE()(dev_state)

#define SET_LIFTER_MOTOR_DIR(dir)(lifter_dir = dir)
#define GET_LIFTER_MOTOR_DIR()(lifter_dir)

#define MIN_XVALUE		50
#define MAX_XVALUE		150
#define MIN_YVALUE		151
#define MAX_YVALUE		251
#define MIN_SPEED	  	10
#define MAX_SPEED	  	19

#define NUM_STEPS	  	1000

#define START_MOVE_RIGHT 	1
#define END_MOVE_RIGHT    	2
#define START_MOVE_LEFT	  	3
#define END_MOVE_LEFT	    4
#define START_UP			5
#define END_UP			    6
#define START_DOWN		    7
#define END_DOWN		    8
#define RELEASE 	        9

#define MAX_BUFFER_l	100

#define PI	3.14159

#define I2C_ADDR_INIT 0x52
#define I2C_ADDR_TOF1 0x40
#define I2C_ADDR_TOF2 0x50
#define I2C_ADDR_TOF3 0x60
#define I2C_ADDR_TOF4 0x70

#define MAX_VBAT		2.7  // after voltage divisor (R1 = 100 Ohms, R2 = 330 Ohms)
#define MIN_VBAT		2.2	// after voltage divisor (R1 = 100 Ohms, R2 = 330 Ohms)

#define TIMER_PERIO_MS	10	// value in ms

#define MIN_DISTANCE_mm	200 // value in mm

#define ACTIVITY_TIMEOUT	5000 //value in seconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId MotorControlHandle;
osThreadId TOFcontrolHandle;
osThreadId LifterControlHandle;
osMutexId timerMutexHandle;
/* USER CODE BEGIN PV */

TimerHandle_t timerHandle;

static uint8_t RxByte_uart1;
static uint8_t RxByte_uart2;

static  float speed;			// Speed limit variable from APP

const float length = 0.175;
const float width  = 0.175;

static  float Vx;
static  float Vy;
static  float Vz;

static  int X;
static  int Y;

int X_ant;
int Y_ant;

int  S1;
int  S2;
int  S3;
int  S4;

static  int distance_1;
static  int distance_2;
static  int distance_3;
static  int distance_4;

static  uint32_t counter;

char buffer[MAX_BUFFER_l];

dev_state_t dev_state = STOP_STATE;

static float vbat_value = 0.00;

//static  uint32_t counter_ms = 0;

uint8_t  message[4];
uint8_t pos_message = 0;

lifter_motor_direction_t lifter_dir = STOP_DIR;

uint8_t distance_flag = 0;

uint8_t block_up_flag = 0;
uint8_t first_up_flag = 1;
uint8_t block_down_flag = 0;
uint8_t first_down_flag = 1;

uint8_t block_right = 0;
uint8_t block_left = 0;
uint8_t block_forward = 0;
uint8_t block_backward = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void MotorMotionControl(void const * argument);
void GetTOFDistance(void const * argument);
void LifterMotionControl(void const * argument);

/* USER CODE BEGIN PFP */

/*Func. prototype for the software timer*/
void vTimerCallback(TimerHandle_t xTimer){
	//Do something
}
int GetAbsValue(int num);

/*Func. prototype for the configuration of vl53l0x sensors*/
HAL_StatusTypeDef configTOF(GPIO_TypeDef* XshutPort, uint16_t XshutPin, uint8_t tofAddr);
static void initSensorsVL53L0X(void);

/* Func. prototype for managment of the ADC*/
static float ReadADClevel(void);

/*Func. prototype for set/reset the ENA driver pins*/
static void enableMovingMotors(void);
static void disableMovingMotors(void);
static void enableLifterMotor(void);
static void disableLifterMotor(void);

/*Function definition for enter state for motor and LEDs managment*/
static void enterStopState(void);
static void enterMovingState(void);
static void enterPacketState(void);

/*Function definition for microstepping for motion motor control*/
static void setMicrostepping(microstepping_t steps);

static void SendReleaseMessage(void);

//int _write(int file, char *ptr, int len)
//{
//	int DataIdx;
//
//	for (DataIdx = 0; DataIdx < len; DataIdx++)
//	{
//		ITM_SendChar(*ptr++);
//	}
//	return len;
//}


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
	X = 0;
	Y = 0;

	X_ant = 0;
	Y_ant = 0;
	Vx = 0;
	Vy = 0;
	Vz = 0;

	speed = 5;

	S1 = 0;
	S2 = 0;
	S3 = 0;
	S4 = 0;

	counter = 0;

	distance_1 = 0;
	distance_2 = 0;
	distance_3 = 0;
	distance_4 = 0;

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /*Init TOF sensors (Change Address, Dist range)*/
   initSensorsVL53L0X(); //  Initialize Time-Of-Flight Sensors

   /*Get voltage value of the VBAT*/
   vbat_value = ReadADClevel();

   /*Set Device State depending of VBAT value*/
   if(IS_VAL_INRANGE(vbat_value, MAX_VBAT, MIN_VBAT)){ //Correct value
 	  enterMovingState();
 	  setMicrostepping(HALF_STEP);
 	  HAL_UART_Receive_IT(&huart1, &RxByte_uart1, 1);
   }else{
	   enterStopState();
   }



  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of timerMutex */
  osMutexDef(timerMutex);
  timerMutexHandle = osMutexCreate(osMutex(timerMutex));

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
  /* definition and creation of MotorControl */
  osThreadDef(MotorControl, MotorMotionControl, osPriorityAboveNormal, 0, 128);
  MotorControlHandle = osThreadCreate(osThread(MotorControl), NULL);

  /* definition and creation of TOFcontrol */
  osThreadDef(TOFcontrol, GetTOFDistance, osPriorityIdle, 0, 128);
  TOFcontrolHandle = osThreadCreate(osThread(TOFcontrol), NULL);

  /* definition and creation of LifterControl */
  osThreadDef(LifterControl, LifterMotionControl, osPriorityIdle, 0, 128);
  LifterControlHandle = osThreadCreate(osThread(LifterControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ENA_2_Pin|ENA_1_Pin|ENA_4_Pin|ENA_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP_3_Pin|DIR_2_Pin|STEP_2_Pin|M2_Pin
                          |M1_Pin|M0_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_4_Pin|STEP_4_Pin|DIR_3_Pin|DIR_5_Pin
                          |STEP_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_1_Pin|STEP_1_Pin|XSHUT_1_Pin|XSHUT_2_Pin
                          |XSHUT_3_Pin|XSHUT_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|ENA_5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ENA_2_Pin ENA_1_Pin STEP_3_Pin DIR_2_Pin
                           STEP_2_Pin M2_Pin M1_Pin M0_Pin
                           LED2_Pin ENA_4_Pin ENA_3_Pin */
  GPIO_InitStruct.Pin = ENA_2_Pin|ENA_1_Pin|STEP_3_Pin|DIR_2_Pin
                          |STEP_2_Pin|M2_Pin|M1_Pin|M0_Pin
                          |LED2_Pin|ENA_4_Pin|ENA_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH1_Pin SWITCH2_Pin */
  GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_4_Pin STEP_4_Pin DIR_3_Pin LED1_Pin
                           DIR_5_Pin STEP_5_Pin ENA_5_Pin */
  GPIO_InitStruct.Pin = DIR_4_Pin|STEP_4_Pin|DIR_3_Pin|LED1_Pin
                          |DIR_5_Pin|STEP_5_Pin|ENA_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_1_Pin STEP_1_Pin XSHUT_1_Pin XSHUT_2_Pin
                           XSHUT_3_Pin XSHUT_4_Pin */
  GPIO_InitStruct.Pin = DIR_1_Pin|STEP_1_Pin|XSHUT_1_Pin|XSHUT_2_Pin
                          |XSHUT_3_Pin|XSHUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){
		if(GET_DEVICE_STATE() == MOVING_STATE){

			if(RxByte_uart1 == START_MOVE_RIGHT){
				Vz = 0.1;

			}else if(RxByte_uart1 == START_MOVE_LEFT){
				Vz = -0.1;

			}else if((RxByte_uart1 == END_MOVE_RIGHT) || (RxByte_uart1 == END_MOVE_LEFT)){
				Vz = 0;

			}else if(IS_VAL_INRANGE(RxByte_uart1, MAX_XVALUE, MIN_XVALUE)){
				Y = CALC_X_VALUE(RxByte_uart1); //x, y inverted
				if(Y > 0){
					if(block_left){
//						Y_ant = Y;
						Y = 0;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					}else{
//						Y = Y_ant;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					}
				}else if(Y < 0){
					if(block_right){
//						Y_ant = Y;
						Y = 0;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					}else{
//						Y = Y_ant;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					}
				}

			}else if(IS_VAL_INRANGE(RxByte_uart1, MAX_YVALUE, MIN_YVALUE)){
				X = CALC_Y_VALUE(RxByte_uart1);
				if(X > 0){
					if(block_forward){
//						X_ant = X;
						X = 0;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					}else{
//						X = X_ant;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					}
				}else if(X < 0){
					if(block_backward){
//						X_ant = X;
						X = 0;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					}else{
//						X = X_ant;
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					}
				}

			}else if(IS_VAL_INRANGE(RxByte_uart1, MAX_SPEED, MIN_SPEED)){
				speed = CALC_SPEED(RxByte_uart1);

			}else if(RxByte_uart1 == START_UP){
				if(first_up_flag){
					enableLifterMotor();
					first_up_flag = 0;
				}
				if(!block_up_flag){
					SET_LIFTER_MOTOR_DIR(UP_DIR);
				}

			}else if(RxByte_uart1 == START_DOWN){
				if(first_down_flag){
					enableLifterMotor();
					first_down_flag = 0;
				}
				if(!block_down_flag){
					SET_LIFTER_MOTOR_DIR(DOWN_DIR);
				}

			}else if((RxByte_uart1 == END_UP) || (RxByte_uart1 == END_DOWN)){
				disableLifterMotor();
				first_down_flag = first_up_flag = 1;
				SET_LIFTER_MOTOR_DIR(STOP_DIR);

			}else if(RxByte_uart1 == RELEASE){
				enterPacketState();
			}
			else{
				X = 0;
				Y = 0;
				speed = 0;
			}
		}else{
			X = Y = 0;
			Vx = Vy = Vz = 0.00;
			speed = 0;
		}
		HAL_UART_Receive_IT(huart, &RxByte_uart1, 1);
	}

	if(huart->Instance == USART2){
		if(GET_DEVICE_STATE() == PACKET_STATE){
			if(RxByte_uart2 == 'F'){
				enterMovingState();
			}
		}

	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == SWITCH1_Pin){
		if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin) == GPIO_PIN_SET){ //Rising edge
			SET_LIFTER_MOTOR_DIR(STOP_DIR);
			block_up_flag = 1;
		}else{  //Falling edge
			block_up_flag = 0;
		}

	}else if(GPIO_Pin == SWITCH2_Pin){
		if(HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin)){  //Rising edge
			SET_LIFTER_MOTOR_DIR(STOP_DIR);
			block_down_flag = 1;
		}else{  //Falling edge
			block_down_flag = 0;
		}


	}

}


int GetAbsValue(int num){
	if(num < 0){
		num = -num;
	}
	return num;
}

HAL_StatusTypeDef configTOF(GPIO_TypeDef* XshutPort, uint16_t XshutPin, uint8_t tofAddr)
{
  HAL_StatusTypeDef Status = HAL_OK;

  HAL_GPIO_WritePin(XshutPort, XshutPin, GPIO_PIN_SET);
//  Status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDR_INIT, 1, HAL_MAX_DELAY);
////  if(Status != HAL_OK){
////    return Status;
////  }

  SetDevAddr(tofAddr, I2C_ADDR_INIT);

  Status = HAL_I2C_IsDeviceReady(&hi2c1, tofAddr, 1, 1000);
  if(Status != HAL_OK){
  	  return Status;
  }

  if(!tofInit(2,tofAddr)){
	  return HAL_ERROR;
  }

  HAL_Delay(500);
  return Status;
}

static void initSensorsVL53L0X(void){
	  HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin|XSHUT_2_Pin|XSHUT_3_Pin|XSHUT_4_Pin, GPIO_PIN_RESET);

	  configTOF(XSHUT_1_GPIO_Port, XSHUT_1_Pin, I2C_ADDR_TOF1);
	  configTOF(XSHUT_2_GPIO_Port, XSHUT_2_Pin, I2C_ADDR_TOF2);
	  configTOF(XSHUT_3_GPIO_Port, XSHUT_3_Pin, I2C_ADDR_TOF3);
	  configTOF(XSHUT_4_GPIO_Port, XSHUT_4_Pin, I2C_ADDR_TOF4);
}

static float ReadADClevel(void){

	uint32_t adc_value = 0;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return (adc_value * 3.3f) / 4096.0f;

}

//void CountTimer_ms(TimerHandle_t xTimer){
//	osMutexWait(timer_mutexHandle, osWaitForever);
//	counter_ms++;
//	osMutexRelease(timer_mutexHandle);
//}
//
//static uint32_t GetTimer_ms(void){
//	osMutexWait(timer_mutexHandle, osWaitForever);
//	uint32_t timer_value = counter_ms;
//	osMutexRelease(timer_mutexHandle);
//	return timer_value;
//
//}

static void enableMovingMotors(void){
	HAL_GPIO_WritePin(ENA_1_GPIO_Port, ENA_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENA_2_GPIO_Port, ENA_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENA_3_GPIO_Port, ENA_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENA_4_GPIO_Port, ENA_4_Pin, GPIO_PIN_RESET);
}

static void disableMovingMotors(void){
	HAL_GPIO_WritePin(ENA_1_GPIO_Port, ENA_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA_2_GPIO_Port, ENA_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA_3_GPIO_Port, ENA_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA_4_GPIO_Port, ENA_4_Pin, GPIO_PIN_SET);
}

static void enableLifterMotor(void){
	HAL_GPIO_WritePin(ENA_5_GPIO_Port, ENA_5_Pin, GPIO_PIN_RESET);
}

static void disableLifterMotor(void){
	HAL_GPIO_WritePin(ENA_5_GPIO_Port, ENA_5_Pin, GPIO_PIN_SET);

}
static void enterStopState(void){
	SET_DEVICE_STATE(STOP_STATE);
	disableMovingMotors();
	disableLifterMotor();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

static void enterMovingState(void){
	SET_DEVICE_STATE(MOVING_STATE);
	enableMovingMotors();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

static void enterPacketState(void){

	SET_DEVICE_STATE(PACKET_STATE);
	disableMovingMotors();
	disableLifterMotor();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	SendReleaseMessage();

}

static void setMicrostepping(microstepping_t steps){

	//Reset microstepping value : FULL_STEP (all MX_pins initialized in reset state)
	switch (steps) {
		case HALF_STEP:
			HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
			break;
		case MICROSTEPPING_1_4:
			HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
			break;
		case MICROSTEPPING_8:
			HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
			break;
		case MICROSTEPPING_16:
			HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_SET);
			break;
		case MICROSTEPPING_32:
			HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}

static void SendReleaseMessage(void){
	uint8_t mssg = 'R';
	HAL_UART_Transmit(&huart2, &mssg, sizeof(mssg), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, &RxByte_uart2, 1);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_MotorMotionControl */
/**
* @brief Function implementing the MotorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorMotionControl */
void MotorMotionControl(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(GET_DEVICE_STATE() == MOVING_STATE){

		  Vx = CALC_VEL(X,speed);
		  Vy = CALC_VEL(Y,speed);

		  S1 += ((1/0.030) * ((-length-width)*((10*speed)*Vz) + (Vx) - (Vy)) * (200./(12*PI)));
		  S2 += ((1/0.030) * ((+length+width)*((10*speed)*Vz) + (Vx) + (Vy)) * (200./(12*PI)));
		  S3 += ((1/0.030) * ((+length+width)*((10*speed)*Vz) + (Vx) - (Vy)) * (200./(12*PI)));
		  S4 += ((1/0.030) * ((-length-width)*((10*speed)*Vz) + (Vx) + (Vy)) * (200./(12*PI)));

		  if(GetAbsValue(S1) >= NUM_STEPS){
			HAL_GPIO_WritePin(STEP_1_GPIO_Port, STEP_1_Pin, GPIO_PIN_SET);
			if(S1 < 0){
			  HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
			  S1 += NUM_STEPS;
		  }else{
			  HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
			  S1 -=  NUM_STEPS;
		  }
		  }
		  if(GetAbsValue(S2) >= NUM_STEPS){
			HAL_GPIO_WritePin(STEP_2_GPIO_Port, STEP_2_Pin, GPIO_PIN_SET);
			if(S2 < 0){
			  HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
			  S2 += NUM_STEPS;
		  }else{
			  HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
			  S2 -=  NUM_STEPS;
		  }
		  }
		  if(GetAbsValue(S3) >= NUM_STEPS){
			HAL_GPIO_WritePin(STEP_3_GPIO_Port, STEP_3_Pin, GPIO_PIN_SET);
			if(S3 < 0){
			  HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_SET);
			  S3 += NUM_STEPS;
		  }else{
			  HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_RESET);
			  S3 -=  NUM_STEPS;
		  }
		  }
		  if(GetAbsValue(S4) >= NUM_STEPS){
			HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_SET);
			if(S4 < 0){
			  HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_RESET);
			  S4 += NUM_STEPS;
			}else{
			  HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_SET);
			  S4 -=  NUM_STEPS;
			}
		  }

		  HAL_GPIO_WritePin(STEP_1_GPIO_Port, STEP_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(STEP_2_GPIO_Port, STEP_2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(STEP_3_GPIO_Port, STEP_3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_RESET);
	  }
	  osDelay(1);



  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GetTOFDistance */
/**
* @brief Function implementing the TOFcontrol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetTOFDistance */
void GetTOFDistance(void const * argument)
{
  /* USER CODE BEGIN GetTOFDistance */
  /* Infinite loop */
  for(;;)
  {
	  if(GET_DEVICE_STATE() == MOVING_STATE){
		  distance_1 = tofReadDistance(I2C_ADDR_TOF1);
		  distance_2 = tofReadDistance(I2C_ADDR_TOF2);
		  distance_3 = tofReadDistance(I2C_ADDR_TOF3);
		  distance_4 = tofReadDistance(I2C_ADDR_TOF4);

		  block_left = (distance_1 < MIN_DISTANCE_mm) ? 1 : 0;
		  block_right = (distance_2 < MIN_DISTANCE_mm) ? 1 : 0;
		  block_backward = (distance_3 < MIN_DISTANCE_mm) ? 1 : 0;
		  block_forward = (distance_4 < MIN_DISTANCE_mm) ? 1 : 0;

	  }
	  osDelay(250);
  }
  /* USER CODE END GetTOFDistance */
}

/* USER CODE BEGIN Header_LifterMotionControl */
/**
* @brief Function implementing the LIfterControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LifterMotionControl */
void LifterMotionControl(void const * argument)
{
  /* USER CODE BEGIN LifterMotionControl */
  /* Infinite loop */
  for(;;)
  {
	  if(GET_DEVICE_STATE() == MOVING_STATE){
		  if(GET_LIFTER_MOTOR_DIR() != STOP_DIR){
			  HAL_GPIO_WritePin(DIR_5_GPIO_Port, DIR_5_Pin, GET_LIFTER_MOTOR_DIR());
			  HAL_GPIO_TogglePin(STEP_5_GPIO_Port, STEP_5_Pin);
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END LifterMotionControl */
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
