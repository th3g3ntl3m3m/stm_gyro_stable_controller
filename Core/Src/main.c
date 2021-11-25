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
#include "stdlib.h"
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

#define PARKING_SPEED_DRIVE 15
#define PARKING_SPEED_PARKING 20;

#define PARKING_MAX 1000

#define SPEED_STEPS_MAX_TIME 0.2
#define STEPS_TO_METERS 0.0088495575221239

#define PLATFORM_Y_MAX 5
#define DUTY_MAX_ANGULAR 0.1
#define DUTY_MAX_LINEAR 0.15
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

float PositionIValue;
float SpeedLinearDemand;
float PositionPID;
float RotationPID;
int32_t StepsLeftPrevious;
int32_t StepsRightPrevious;
unsigned long StepsLeftPreviousTime;
unsigned long StepsRightPreviousTime;
float LeftSpeed;
float RightSpeed;
float SpeedPID;
float PlatformYDemand = 0;
float GyroY;
float GyroYPrevious;
float GyroYSpeed;
float GyroZPrevious;
float GyroZSpeed;
float BalancePID;
float DutyFront;
float DutyTurn;
float ResultLeft;
float ResultRight;
float PositionLinearDemand;
float PositionAngularDemand;

float PositionI = 0.0;
float PositionP = 0.6;
float RotationP = 0.004;
float RotationD = 0.01;

float SpeedPNew = 9;
float SpeedINew = 1.5;
float SpeedDNew = 0.7;

float AngleCorrection = 0;
float ParkingAngle;

float BalanceP = 1000;
float BalanceD = 30000;

float ManualDrive = 0;
float RotationI = 0;
float BalanceFilter = 0.9;
float SpeedFilter = 0.015;
float linearIntegral = 0;
float linearLastError = 0;
float linearFcutDiff = 40.0f;
float linearSmoothDiff = 0;
uint32_t linearTime = 0;
float linearDeltaTimePrev = 0;
float minLinearValue = 0.05;
int linearIntegralerrorCoun = 0;

uint8_t PositionLinearControlSwitch;
uint8_t BalanceActive;
int16_t ParkingPersentage;
uint8_t FootAngleDemand;
uint8_t FootDrive;
uint8_t FootParking;
uint8_t FootAngle;
uint8_t FootActive;

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

float SpeedLinear;
float LeftSpeed;
float RightSpeed;
float PositionLinear;

uint8_t BalanceActiveDemand;
uint8_t BTBalanceActive;

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

	if (Battery < 4)
	{
		Front = 0;
		Turn = 0;
	}

	if ((fabsf(Front) < 0.001) && (fabsf(Turn) < 0.001) && (fabsf(SpeedLinear) < 0.02) && (fabsf(LeftSpeed - RightSpeed) < 0.02))
	{
		if ((Battery < 4) && (BalanceActiveDemand))
		{
			BalanceActiveDemand = false;
		}
		else if (Battery > 8)
		{
			BalanceActiveDemand = BTBalanceActive;
		}
		else if (!BTBalanceActive)
		{
			BalanceActiveDemand = false;
		}
	}

	Turn = (Turn > 90) ? 90 : Turn;
	Turn = (Turn < -90) ? -90 : Turn;
	Front = (Front > 2) ? 2 : Front;
	Front = (Front < -0.4) ? -0.4 : Front;

	// Point to add IK sensor

	// Point to add ParkingMode

	BalanceActive = BTBalanceActive;
	PositionLinearDemand = PositionLinear;
}

void BALANCE_Calculate_Speeds()
{
	//LEFT
	float TimeS = (HAL_GetTick() - StepsLeftPreviousTime) / 1000.0;
	if (TimeS > SPEED_STEPS_MAX_TIME)
	{
		TimeS = SPEED_STEPS_MAX_TIME;
	    LeftSpeed = 0;
	}
	if (HallLeftStep != StepsLeftPrevious)
	{
		LeftSpeed = ((HallLeftStep - StepsLeftPrevious) * STEPS_TO_METERS) / TimeS;
	    StepsLeftPrevious = HallLeftStep;
	    StepsLeftPreviousTime = HAL_GetTick();
	}

	//RIGHT
	TimeS = (HAL_GetTick() - StepsRightPreviousTime) / 1000.0;
	if (TimeS > SPEED_STEPS_MAX_TIME)
	{
		TimeS = SPEED_STEPS_MAX_TIME;
	    RightSpeed = 0;
	}
	if (HallRightStep != StepsRightPrevious)
	{
		RightSpeed = ((HallRightStep - StepsRightPrevious) * STEPS_TO_METERS) / TimeS;
	    StepsRightPrevious = HallRightStep;
	    StepsRightPreviousTime = HAL_GetTick();
	}

	PositionLinear = ((HallLeftStep + HallRightStep) / 2) * STEPS_TO_METERS;
	SpeedLinear = (LeftSpeed + RightSpeed) / 2.0;
}

void BALANCE_Position_Linear_Control()
{
	if (fabsf(Front) > 0.001)
	{
		PositionLinearControlSwitch = 0;
	}
	else if (fabsf(SpeedLinear) < 0.02)
	{
		PositionLinearControlSwitch = 1;
	}

	if (BalanceActive)
	{
		float Error = PositionLinearDemand - PositionLinear;
	    if (Error > 0)
	    {
	    	PositionIValue += PositionI;
	    }
	    else
	    {
	    	PositionIValue -= PositionI;
	    }

	    PositionPID = Error * PositionP + PositionIValue;
	    if (PositionLinearControlSwitch)
	    {
	    	SpeedLinearDemand = PositionPID;
	    }
	    else
	    {
	        SpeedLinearDemand = Front;
	        PositionLinearDemand = PositionLinear;
	        PositionIValue = 0;
	    }
	}
	else
	{
		PositionLinearDemand = PositionLinear;
	    PositionIValue = 0;
	}
}

void BALANCE_Speed_LinearControl()
{
	float deltaTime = (HAL_GetTick() - linearTime) / 1000000.0;
	linearTime = HAL_GetTick();

	if (deltaTime < 0)
	{
		deltaTime = linearDeltaTimePrev;
	}
	linearDeltaTimePrev = deltaTime;

	SpeedLinearDemand = SpeedLinearDemand > PLATFORM_Y_MAX ? PLATFORM_Y_MAX : SpeedLinearDemand;
	SpeedLinearDemand = SpeedLinearDemand < -PLATFORM_Y_MAX ? -PLATFORM_Y_MAX : SpeedLinearDemand;

	if (BalanceActive)
	{
	    float linearError = SpeedLinearDemand - SpeedLinear;

	    if ((SpeedLinearDemand * linearIntegral > 0) && (abs(SpeedLinear) > abs(SpeedLinearDemand * 1.5)))
	    {
	        linearIntegralerrorCoun++;
	    }
	    else
	    {
	        linearIntegralerrorCoun = 0;
	    }
	    float diff = (linearError - linearLastError) / deltaTime;
	    linearLastError = linearError;
	    float RC = 1.0f / linearFcutDiff;
	    float kExp = deltaTime / (RC + deltaTime);
	    linearSmoothDiff = (1.0f - kExp) * linearSmoothDiff + kExp * diff;

	    linearIntegral += linearError * deltaTime;
	    if ((SpeedLinearDemand < minLinearValue) && (SpeedLinearDemand > -minLinearValue))
	    {
	        linearIntegral = 0;
	    }

	    if (linearIntegralerrorCoun > 3)
	    {
	        linearIntegral = 0;
	    }
	    SpeedPID = linearError * SpeedPNew + linearIntegral * SpeedINew + linearSmoothDiff * SpeedDNew;

	    PlatformYDemand += ((SpeedPID / 1.0) - PlatformYDemand) * SpeedFilter;

	    PlatformYDemand = (PlatformYDemand > PLATFORM_Y_MAX) ? PLATFORM_Y_MAX : PlatformYDemand;
	    PlatformYDemand = (PlatformYDemand < -PLATFORM_Y_MAX) ? -PLATFORM_Y_MAX : PlatformYDemand;
	}
	else
	{
	    SpeedLinearDemand = 0;
	    linearIntegral = 0;
	}
}

void BALANCE_Position_Angular_Control()
{
	GyroZSpeed = eulerAngles.angle.yaw - GyroZPrevious;
	GyroZPrevious = eulerAngles.angle.yaw;
	if (BalanceActive)
	{
	    PositionAngularDemand -= Turn / 100.0;

	    RotationPID = (eulerAngles.angle.yaw - PositionAngularDemand) * RotationP + GyroZSpeed * RotationD;
	    DutyTurn = RotationPID;
	}
	else
	{
	    PositionAngularDemand = eulerAngles.angle.yaw;
	    DutyTurn = 0;
	}
	DutyTurn = (DutyTurn > DUTY_MAX_ANGULAR) ? DUTY_MAX_ANGULAR : DutyTurn;
	DutyTurn = (DutyTurn < -DUTY_MAX_ANGULAR) ? -DUTY_MAX_ANGULAR : DutyTurn;
}

void BALANCE_LOOP()
{
	GyroY = eulerAngles.angle.pitch + PlatformYDemand + AngleCorrection - ParkingAngle;

	GyroYSpeed = GyroY - GyroYPrevious;
	GyroYPrevious = GyroY;

	if (BalanceActive)
	{
	    BalancePID = -GyroY * (float)BalanceP - GyroYSpeed * (float)BalanceD;

	    DutyFront += BalancePID * 0.000001;

	    DutyFront = (DutyFront > DUTY_MAX_LINEAR) ? DUTY_MAX_LINEAR : DutyFront;
	    DutyFront = (DutyFront < -DUTY_MAX_LINEAR) ? -DUTY_MAX_LINEAR : DutyFront;
	}
	else
	{
	    PlatformYDemand = 0;
	    DutyFront = 0;
	}
}

void BALANCE_Result_Loop()
{
	ResultLeft += ((DutyFront + DutyTurn) - ResultLeft) * BalanceFilter;
	ResultRight += ((DutyFront - DutyTurn) - ResultRight) * BalanceFilter;

	SerialControlWheelsRequest.WheelLeft = ResultLeft;
	SerialControlWheelsRequest.WheelRight = ResultRight;

	SerialControlWheelsRequest.WheelLeft += RotationI * (BTFront);
	SerialControlWheelsRequest.WheelRight += RotationI * (BTFront);
	SerialControlWheelsRequest.WheelLeft += ManualDrive;
	SerialControlWheelsRequest.WheelRight += ManualDrive;

	if (eulerAngles.angle.pitch + AngleCorrection > 20)
	{
	    SerialControlWheelsRequest.WheelLeft = 0;
	    SerialControlWheelsRequest.WheelRight = 0;
	}
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
			  HallLeftStepPast = HallLeftStep;
		  }
		  else
		  {
			  HallLeftStep = HallLeftStepPast;
		  }

		  if (HallActualize(SerialControlWheelsResponce.WheelRightSteps, HallRightStepPast, MOTHERBOARD_DIFF))
		  {
			  HallRightStep = SerialControlWheelsResponce.WheelRightSteps;
			  HallRightStepPast = HallRightStep;
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

	  BALANCE_Prepare();
	  BALANCE_Calculate_Speeds();
	  BALANCE_Speed_LinearControl();
	  BALANCE_Position_Angular_Control();
	  BALANCE_LOOP();
	  BALANCE_Result_Loop();

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
