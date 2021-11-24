/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "Fusion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)
#define WHEELS_REQUEST_SIZE 16 // Пакет для взаимодействия с STMF103 на плате гироскутера
union {
    struct
    {
        uint8_t ControlMode;
        uint8_t ParameterNumber;
        float ParameterValue;
        float WheelLeft;
        float WheelRight;
        uint8_t CR;
        uint8_t LF;
    };
    uint8_t Buffer[WHEELS_REQUEST_SIZE];
} SerialControlWheelsRequest;

#define WHEELS_RESPONCE_SIZE 16
union {
    struct
    {
        uint8_t State;
        uint8_t ParameterNumber;
        float ParameterValue;
        int32_t WheelLeftSteps;
        int32_t WheelRightSteps;
        uint8_t CR;
        uint8_t LF;
    };
    uint8_t Buffer[WHEELS_RESPONCE_SIZE];
} SerialControlWheelsResponce;

#define ON_BOARD_CONTROL_REQUEST_SIZE 10
union {
	struct
	{
		float Linear;
		int16_t Angular;
		int8_t ParkingMode;
		uint8_t LedMode;
		uint8_t CR;
		uint8_t LF;
	};
	uint8_t Buffer[ON_BOARD_CONTROL_REQUEST_SIZE];
} SerialOnBoardRequest;

#define ON_BOARD_CONTROL_RESPONCE_SIZE 10
union {
	struct
	{
		int32_t WheelLeftSteps;
		int32_t WheelRightSteps;
		uint8_t CR;
		uint8_t LF;
	};
	uint8_t Buffer[ON_BOARD_CONTROL_RESPONCE_SIZE];
}SerialOnBoardResponce;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_INIT 0

#define ON_BOARD_PC_DELAY_MS 100
#define MOTHERBOARD_DELAY_MS 100

#define MOTHERBOARD_DIFF 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t USART1ReceiveState=0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good  //OnBoard Plate
volatile uint8_t USART2ReceiveState=0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good  //STM Plate

float Voltage;
float Battery;
float CurrentLeft;
float CurrentRight;
float RPSLeft;
float RPSRight;
float OverCurrCount;
float ConnErrCount;
float CommTime;

uint8_t* LostByte;

uint32_t PackageLastTimeReset_Motherboard;
uint32_t PackageLastTimeReset_OnBoardPC;

int ControlCommandTimeoutMS = 2000;

float HallLeftStepPast = 0;
float HallRightStepPast = 0;
float HallLeftStep;
float HallRightStep;

uint8_t ParameterNumber;

float BTFront = 0;
float BTTurn = 0;
float Front = 0;
float Turn = 0;

axises my_gyro;
axises my_accel;
axises my_mag;

FusionBias fusionBias;
FusionAhrs fusionAhrs;
float samplePeriod = 0.1f;
FusionVector3 gyroscopeSensitivity;
FusionVector3 accelerometerSensitivity;
FusionVector3 hardIronBias;
FusionVector3 uncalibratedGyroscope;
FusionVector3 uncalibratedAccelerometer;
FusionVector3 uncalibratedMagnetometer;
FusionEulerAngles eulerAngles;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_StatusTypeDef Res;

	if (UartHandle->Instance == UART4){ // Jetson commutation
		if (USART1ReceiveState == 0){
			if ((SerialOnBoardRequest.CR != 13) || (SerialOnBoardRequest.LF != 10)){
				Res = HAL_UART_Receive_DMA(&huart4, LostByte, 1);
				USART1ReceiveState = 1;
			}
			else{
 				USART1ReceiveState = 10;
				Res = HAL_UART_Receive_DMA(&huart4, (uint8_t*)SerialOnBoardRequest.Buffer, ON_BOARD_CONTROL_REQUEST_SIZE);
			}
		}
		else{
			if(USART1ReceiveState == 1){
				if (LostByte[0] == 13){
					USART1ReceiveState = 2;
				}
				Res = HAL_UART_Receive_DMA(&huart4, (uint8_t*)LostByte, 1);
			}
			else{
				if (USART1ReceiveState == 2){
					if (LostByte[0] == 10){
						USART1ReceiveState = 0;
						Res = HAL_UART_Receive_DMA(&huart4, (uint8_t*)SerialOnBoardRequest.Buffer, ON_BOARD_CONTROL_REQUEST_SIZE);
					}
					else{
						USART1ReceiveState = 1;
						Res = HAL_UART_Receive_DMA(&huart4, (uint8_t*)LostByte, 1);
					}
				}
			}
		}
		if (Res != HAL_OK)
		{
			MX_UART4_Init();
			USART1ReceiveState = 0;
			Res = HAL_UART_Receive_DMA(&huart4, (uint8_t*)SerialOnBoardRequest.Buffer, ON_BOARD_CONTROL_REQUEST_SIZE);
		}
	}

	if (UartHandle->Instance == USART2)
	{
		if (USART2ReceiveState == 0)
		{
			if ((SerialControlWheelsResponce.CR != 13) || (SerialControlWheelsResponce.LF != 10))
			{
				Res = HAL_UART_Receive_DMA(&huart2, LostByte, 1);
				USART2ReceiveState = 1;
			}
			else
			{
				USART2ReceiveState = 10;
				Res = HAL_UART_Receive_DMA(&huart2, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
			}
		}
		else
		{
			if(USART2ReceiveState == 1)
			{
				if (LostByte[0] == 13)
				{
					USART2ReceiveState = 2;
				}
				Res = HAL_UART_Receive_DMA(&huart2, (uint8_t*)LostByte, 1);
			}
			else
			{
				if (USART2ReceiveState == 2)
				{
					if (LostByte[0] == 10)
					{
						USART2ReceiveState = 0;
						Res = HAL_UART_Receive_DMA(&huart2, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
					}
					else
					{
						USART2ReceiveState = 1;
						Res = HAL_UART_Receive_DMA(&huart2, (uint8_t*)LostByte, 1);
					}
				}
			}
		}

		if (Res != HAL_OK)
		{
			MX_USART2_UART_Init();
			USART2ReceiveState = 0;
			Res = HAL_UART_Receive_DMA(&huart2, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
		}
	}
}
int HallActualize(float NewStep, float LastStep, float difference)
{
	float MIN_VAL = LastStep - difference;
	float MAX_VAL = LastStep + difference;

	if ((NewStep < MAX_VAL) && (NewStep > MIN_VAL))
	{
		return 1;
	}
	return 0;
}
void IMU_INIT()
{
	gyroscopeSensitivity.axis.x = 1.0f;
	gyroscopeSensitivity.axis.y = 1.0f;
	gyroscopeSensitivity.axis.z = 1.0f;

	accelerometerSensitivity.axis.x = 1.0f;
	accelerometerSensitivity.axis.y = 1.0f;
	accelerometerSensitivity.axis.z = 1.0f;

	hardIronBias.axis.x = 0.0f;
	hardIronBias.axis.y = 0.0f;
	hardIronBias.axis.z = 0.0f;

	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod);
	FusionAhrsInitialise(&fusionAhrs, 0.5f);
}
void IMU_UPDATE()
{
	icm20948_gyro_read_dps(&my_gyro);
	icm20948_accel_read_g(&my_accel);
	ak09916_mag_read_uT(&my_mag);

	uncalibratedGyroscope.axis.x = my_gyro.x;
	uncalibratedGyroscope.axis.y = my_gyro.y;
	uncalibratedGyroscope.axis.z = my_gyro.z;

	uncalibratedAccelerometer.axis.x = my_accel.x;
	uncalibratedAccelerometer.axis.y = my_accel.y;
	uncalibratedAccelerometer.axis.z = my_accel.z;

	uncalibratedMagnetometer.axis.x = my_mag.x;
	uncalibratedMagnetometer.axis.y = my_mag.y;
	uncalibratedMagnetometer.axis.z = my_mag.z;

	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);
	FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);
	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);
	FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);
	eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
}
void SERIAL_CONTROL_LOOP()
{
	SerialControlWheelsRequest.ControlMode = 0;
	SerialControlWheelsRequest.ParameterNumber = 0;
	SerialControlWheelsRequest.WheelLeft = 0.0;
	SerialControlWheelsRequest.WheelRight = 0.0;
	SerialControlWheelsRequest.CR=13;
	SerialControlWheelsRequest.LF=10;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)SerialControlWheelsRequest.Buffer, WHEELS_REQUEST_SIZE);
}
void BALANCE_Prepare()
{
	Front = BTFront;
	Turn = BTTurn;
}
float Interpolation(float Value, float Min, float Max)
{
    float Result = (Value - Min) / (Max - Min);
    if (Result > 1)
    {
        return 1;
    }
    if (Result < 0)
    {
        return 0;
    }
    return Result;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  icm20948_init();
  ak09916_init ();
  IMU_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  IMU_UPDATE();

	  if (HAL_GetTick() - PackageLastTimeReset_Motherboard > 100) // UART2 RECEIVE FEEDBACK
	  {
		  MX_USART2_UART_Init();
		  USART2ReceiveState = 0;
		  HAL_UART_Receive_DMA(&huart2, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
		  PackageLastTimeReset_Motherboard = HAL_GetTick();
	  }

	  if ((USART2ReceiveState == 10) && (SerialControlWheelsResponce.CR == 13) && (SerialControlWheelsResponce.LF == 10))
	  {
		  USART2ReceiveState = 0;
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

		  if (HallActualize(SerialControlWheelsResponce.WheelLeftSteps, HallLeftStepPast, MOTHERBOARD_DIFF))
		  {
			  HallLeftStep = SerialControlWheelsResponce.WheelLeftSteps;
		  }
		  else
		  {
			  HallLeftStep = HallLeftStepPast;
		  }

		  if (HallActualize(SerialControlWheelsResponce.WheelRightSteps, HallRightStepPast, MOTHERBOARD_DIFF))
		  {
			  HallRightStep = SerialControlWheelsResponce.WheelRightSteps;
		  }
		  else
		  {
			  HallRightStep = HallRightStepPast;
		  }

		  PackageLastTimeReset_Motherboard = HAL_GetTick();

		  switch (SerialControlWheelsResponce.ParameterNumber)
		  {
		  case 0:
			  Voltage = SerialControlWheelsResponce.ParameterValue;
			  Battery += ((Interpolation(Voltage, 28, 41) * 100.0) - Battery) * 0.01;
			  break;
		  case 1:
			  CurrentLeft = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 2:
		      CurrentRight = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 3:
		      RPSLeft = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 4:
		      RPSRight = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 5:
		      OverCurrCount = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 6:
		      ConnErrCount = SerialControlWheelsResponce.ParameterValue;
		      break;
		  case 7:
		      CommTime = SerialControlWheelsResponce.ParameterValue;
		      break;
		  }
	  }

	  /*if ((USART1ReceiveState == 10) && (SerialOnBoardRequest.CR == 13) && (SerialOnBoardRequest.LF == 10))
	  {
		  USART1ReceiveState = 0;
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

		  if ((SerialOnBoardRequest.Linear <= 0.3) && (SerialOnBoardRequest.Linear >= -0.3))
		  {
			  Left = SerialOnBoardRequest.Linear;
			  Right = SerialOnBoardRequest.Linear;
		  }

		  SerialOnBoardResponce.WheelLeftSteps = SerialControlWheelsResponce.WheelLeftSteps;
		  SerialOnBoardResponce.WheelRightSteps = SerialControlWheelsResponce.WheelRightSteps;
		  SerialOnBoardResponce.CR = 13;
		  SerialOnBoardResponce.LF = 10;

 		  HAL_UART_Transmit_DMA(&huart4, (uint8_t*)SerialOnBoardResponce.Buffer, ON_BOARD_CONTROL_RESPONCE_SIZE);

		  PackageLastTimeReset_OnBoardPC = HAL_GetTick();
	  }*/

	  SERIAL_CONTROL_LOOP();

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
