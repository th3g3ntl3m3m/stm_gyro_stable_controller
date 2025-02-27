/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * 					<h2><center>RTK-WOLVES</center></h2>
  *
  *
  * 								Created by
  * 						@Lord_tachanka @th3d0l0rh3z3
  *                        			  AT 2021
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
#include "icm20948.h"	// Драйвер для гироскопа (самопис.)
#include "Fusion.h"		// Библиотека преобразований данных гироскопа (самопис.)
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)
#define WHEELS_REQUEST_SIZE 16 				// Размер пакета
union {
    struct
    {
        uint8_t ControlMode;				// 1 - Режим управления(не используется)
        uint8_t ParameterNumber;			// 1 - Номер параметра(не используется)
        float ParameterValue;				// 4 - Значение параметра(не используется)
        float WheelLeft;					// 4 - Ш�?М левое колесо, от -1 до 1
        float WheelRight;					// 4 - Ш�?М правое колесо, от -1 до 1
        uint8_t CR;							// 1 - Байт синхронизации
        uint8_t LF;							// 1 - Байт синхронизации
    };
    uint8_t Buffer[WHEELS_REQUEST_SIZE];	// Буффер байт
} SerialControlWheelsRequest;				// Пакет для отправки на плату гироскутера

#define WHEELS_RESPONCE_SIZE 16				// Размер пакета
union {
    struct
    {
        uint8_t State;						// 1 - Код статуса
        uint8_t ParameterNumber;			// 1 - Номер параметра
        float ParameterValue;				// 4 - Значение параметра
        int32_t WheelLeftSteps;				// 4 - Положение по датчикам холла, левое колесо
        int32_t WheelRightSteps;			// 4 - Положение по датчикам холла, правое колесо
        uint8_t CR;							// 1 - Байт синхронизации
        uint8_t LF;							// 1 - Байт синхронизации
    };
    uint8_t Buffer[WHEELS_RESPONCE_SIZE];	// Буффер байт
} SerialControlWheelsResponce;				// Пакет приема ответного сообщения с платы гироскутера

#define HIGH_LEVEL_REQUEST_SIZE 16				// Размер пакета
union {
	struct
	{
		float Linear;			 				// 4 - Линейная скорость
		float Angular;							// 4 - Угловая скорость
		int8_t DriveMode;						// 1 - Режим движения
		uint8_t ParameterNumber;				// 1 - �?ндекс параметров
		float ParametrValue;					// 4 - Значние параметра
		uint8_t CR;								// 1 - Байт синхронизации
		uint8_t LF;								// 1 - Байт синхронизации
	};
	uint8_t Buffer[HIGH_LEVEL_REQUEST_SIZE];	// Буффер байт
} SerialHighLevelRequest;						// Принимаемый пакет от вычислителя вехнего уровня

#define HIGH_LEVEL_RESPONCE_SIZE 25				// Размер пакета
union {
	struct
	{
		int8_t ControllerState;					// 1 - Статусное состояние контроллера
		int32_t WheelLeftSteps;					// 4 - Показания датчика холла, левый
		int32_t WheelRightSteps;				// 4 - Показания датчика холла, правый
		uint8_t BatteryPersentage;				// 1 - Процент заряда батареи
		int16_t Roll;							// 2 - Крен (AHRS)
		int16_t Pitch;							// 2 - Тангаж (AHRS)
		int16_t Yaw;							// 2 - Рысканье (AHRS)
		uint16_t CenterIkSensor;				// 2 - Показания �?К-дальномера
		uint8_t ParameterNumber;				// 1 - �?ндекс параметров
		float ParametrValue;					// 4 - Номер параметра
		uint8_t CR;								// 1 - Байт синхронизации
		uint8_t LF;								// 1 - Байт синхронизации
	};
	uint8_t Buffer[HIGH_LEVEL_RESPONCE_SIZE];	// Буфер байт
}SerialHighLevelResponce;						// Отправляемый в ответ пакет
#pragma pack(pop)

typedef struct
{
	int32_t LastHall;
	int32_t OutputHall;
} HallFilter;

typedef struct
{
	float Voltage;
	float Battery;
	float CurrentLeft;
	float CurrentRight;
	float RPSLeft;
	float RPSRight;
	float OverCurrCount;
	float ConnErrCount;
	float CommTime;
} LowUartData;

typedef struct
{
	uint16_t* Raw;
	uint16_t* Sensors;
	float* Amperage;
} ADCData;

typedef struct
{
	float Front;
	float Turn;
	float Drive;
} InputControl;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define SYSTEM_NO_ADC_INIT
//#define SYSTEM_NO_IMU_HW_INIT

//#define SYSTEM_NO_LOW_UART_LOOP
//#define SYSTEM_NO_HIGH_UART_LOOP
//#define SYSTEM_NO_GPIO_LOOP
//#define SYSTEM_NO_ADC_LOOP

//#define DEBUG_NO_ADC_ALL
//#define DEBUG_NO_ADC_RAW
//#define DEBUG_NO_ADC_SEN
//#define DEBUG_NO_ADC_AMP

#define SYSTEM_HARDWARE_UART_LOW (&huart2)
#define SYSTEM_HARDWARE_UART_LOW_INSTANSE USART2
#define SYSTEM_HARDWARE_UART_HIGH (&huart3)
#define SYSTEM_HARDWARE_UART_HIGH_INSTANSE USART3
#define SYSTEM_HARDWARE_PARKING_LEG_UP_PORT GPIOE
#define SYSTEM_HARDWARE_PARKING_LEG_UP_PIN GPIO_PIN_5
#define SYSTEM_HARDWARE_PARKING_LEG_DOWN_PORT GPIOE
#define SYSTEM_HARDWARE_PARKING_LEG_DOWN_PIN GPIO_PIN_4
#define SYSTEM_HARDWARE_ADC (&hadc1)
#define SYSTEM_HARDWARE_ADC_IK_FL ADC_CHANNEL_11
#define SYSTEM_HARDWARE_ADC_IK_FR ADC_CHANNEL_12
#define SYSTEM_HARDWARE_ADC_IK_BL ADC_CHANNEL_1
#define SYSTEM_HARDWARE_ADC_IK_BR ADC_CHANNEL_10
#define SYSTEM_HARDWARE_ADC_IK_CN ADC_CHANNEL_2
#define SYSTEM_HARDWARE_ADC_AMP_36 ADC_CHANNEL_9
#define SYSTEM_HARDWARE_ADC_AMP_12 ADC_CHANNEL_8
#define SYSTEM_HARDWARE_ADC_AMP_5 ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_Channel_Count 8
#define SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PORT GPIOC
#define SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PIN GPIO_PIN_4
#define SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PORT GPIOC
#define SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PIN GPIO_PIN_5
#define SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PORT GPIOE
#define SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PIN GPIO_PIN_8

#define SYSTEM_TIMING_MS_UART_LOW 100
#define SYSTEM_TIMING_MS_UART_HIGH 100
#define SYSTEM_TIMING_MS_GPIO 100
#define SYSTEM_TIMING_MS_ADC 10
#define SYSTEM_TIMING_MS_IMU 10
#define SYSTEM_TIMING_MS_LOGIC 10

#define SYSTEM_HALL_FILTER_MAX 1000
#define SYSTEM_IMU_ACCEL_FILTER 0.1
#define SYSTEM_IMU_GYRO_FILTER 0.1
#define SYSTEM_IMU_MAG_FILTER 0.1

#define BALANCE_SPEED_STEPS_MAX_TIME 0.2
#define BALANCE_STEPS_TO_METERS 0.0088495575221239
#define BALANCE_PLATFORM_Y_MAX 5
#define BALANCE_DUTY_MAX_ANGULAR 0.1
#define BALANCE_DUTY_MAX_LINEAR 0.2
/*
#define SYSTEM_HARDWARE_PWM_CH1_PORT (&htim1)
#define SYSTEM_HARDWARE_PWM_CH1_CH TIM_CHANNEL_1
#define SYSTEM_HARDWARE_PWM_CH2_PORT (&htim1)
#define SYSTEM_HARDWARE_PWM_CH2_CH TIM_CHANNEL_2
#define SYSTEM_HARDWARE_PWM_CH3_PORT (&htim1)
#define SYSTEM_HARDWARE_PWM_CH3_CH TIM_CHANNEL_3
#define SYSTEM_HARDWARE_PWM_CH4_PORT (&htim4)
#define SYSTEM_HARDWARE_PWM_CH4_CH TIM_CHANNEL_2
#define SYSTEM_HARDWARE_LED1_PORT GPIOD
#define SYSTEM_HARDWARE_LED1_PIN GPIO_PIN_12
#define SYSTEM_HARDWARE_LED2_PORT GPIOD
#define SYSTEM_HARDWARE_LED2_PIN GPIO_PIN_13
#define SYSTEM_HARDWARE_LED3_PORT GPIOD
#define SYSTEM_HARDWARE_LED3_PIN GPIO_PIN_14
#define SYSTEM_HARDWARE_LED4_PORT GPIOD
#define SYSTEM_HARDWARE_LED4_PIN GPIO_PIN_15

#define SYSTEM_STATE_INIT 0

#define SYSTEM_DELAY_MS_HIGH_UART 100
#define SYSTEM_DELAY_MS_LOW_UART 100

#define WS2812_DELAY_LEN 48
*/
#define DELAY_LEN 48
#define LED_COUNT 27
#define HIGH 65
#define LOW 26
#define ARRAY_LEN DELAY_LEN+LED_COUNT*24
#define BitIsSet(reg, bit) ((reg & (1<<bit))!=0)
#define MAX_BRIGHTNESS 50
uint32_t BUF_DMA[ARRAY_LEN]={0};

uint16_t MAX_LIGHT = 128;
/*

#define MOTHERBOARD_DIFF 100

#define PARKING_SPEED_DRIVE 15
#define PARKING_SPEED_PARKING 20;

#define PARKING_MAX 1000

#define SPEED_STEPS_MAX_TIME 0.2

#define ADC_CH_COUNT 8
#define STEPPER_DRIVE_MAX 3200
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t LastPkgTimeUartLow = 0;
uint32_t LastPkgTimeUartHigh = 0;
volatile uint8_t UartLowReceiveState = 0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good
volatile uint8_t UartHighReceiveState = 0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good
uint8_t* LostByte;
HallFilter WheelsHall[2];
uint8_t InitionHall = 0;
LowUartData LowDiagnostic;

uint8_t FootButtonUp = 0;
uint8_t FootButtonDown = 0;

uint32_t LastUpdateGPIO = 0;
uint32_t LastUpdateADC = 0;
uint32_t LastUpdateIMU = 0;
uint32_t LastUpdateLogic = 0;
uint32_t LastUpdateLed = 0;

ADCData AdcModule;

axises ResGyro;
axises ResAccel;
axises ResMag;
FusionBias fusionBias;
FusionAhrs fusionAhrs;
float samplePeriod = 0.01f;
FusionVector3 gyroscopeSensitivity;
FusionVector3 accelerometerSensitivity;
FusionVector3 hardIronBias;
FusionVector3 uncalibratedGyroscope;
FusionVector3 uncalibratedAccelerometer;
FusionVector3 uncalibratedMagnetometer;
FusionEulerAngles eulerAngles;

InputControl BTControl;

uint8_t FootDone = 0;

// balance ===========
uint8_t BalanceActive;
float PositionLinearDemand;
// Prepare
float Front = 0;
float Turn = 0;
float SpeedLinear;
float LeftSpeed;
float RightSpeed;
uint8_t BalanceActiveDemand;
// CalcSpeeds
float TimeS;
unsigned long StepsLeftPreviousTime;
unsigned long StepsRightPreviousTime;
int32_t StepsLeftPrevious;
int32_t StepsRightPrevious;
float SpeedLinear;
float SpeedLinearDemand;
float PositionLinear;
//PosControl
uint8_t PositionLinearControlSwitch;
float PositionIValue;
float PositionI = 0.0;
float PositionP = 0.6;
float PositionPID;
uint32_t linearTime = 0;
float linearDeltaTimePrev = 0;
float linearIntegral = 0;
int linearIntegralerrorCoun = 0;
float linearLastError = 0;
float linearFcutDiff = 40.0f;
float linearSmoothDiff = 0;
float minLinearValue = 0.05;
float SpeedPID;
float SpeedPNew = 9;
float SpeedINew = 1.5;
float SpeedDNew = 0.7;
float PlatformYDemand = 0;
float SpeedFilter = 0.015;
float AngleCorrection = 3;
float BalanceP = 100;
float BalanceD = 15000;
float BalanceFilter = 0.99;
float BalancePID;
float GyroY;
float GyroYPrevious;
float GyroYSpeed;
float GyroZPrevious;
float GyroZSpeed;
float DutyFront;
float DutyTurn;
float ParkingAngle;
float ResultLeft;
float ResultRight;
float PositionAngularDemand;
float RotationPID;
float RotationP = 0.004;
float RotationI = 0;
float RotationD = 0.01;

float ManualDrive = 0;

//debug
#ifndef DEBUG_NO_ADC_ALL
#ifndef DEBUG_NO_ADC_RAW
uint16_t DebugADCRawFL;
uint16_t DebugADCRawFR;
uint16_t DebugADCRawBL;
uint16_t DebugADCRawBR;
uint16_t DebugADCRawCN;
uint16_t DebugADCRaw36;
uint16_t DebugADCRaw12;
uint16_t DebugADCRaw5;
#endif
#ifndef DEBUG_NO_ADC_SEN
uint16_t DebugADCSenFL;
uint16_t DebugADCSenFR;
uint16_t DebugADCSenBL;
uint16_t DebugADCSenBR;
uint16_t DebugADCSenCN;
#endif
#ifndef DEBUG_NO_ADC_AMP
float DebugADCAmp36;
float DebugADCAmp12;
float DebugADCAmp5;
#endif
#endif

axises DebugAccRaw;
axises DebugGyroRaw;
float DebugFilterAcc = 0.3;
float DebugScallerAcc = 1000;
float DebugFilterGyro = 0.4;
float DebugScallerGyro = 1000;
/*

float LeftSpeed;
float RightSpeed;

float linearIntegral = 0;

float linearFcutDiff = 40.0f;
float linearSmoothDiff = 0;

int linearIntegralerrorCoun = 0;

uint32_t CurrentStepPark = 0;

uint8_t PositionLinearControlSwitch;
uint8_t BalanceActive;
int16_t ParkingPersentage;
uint8_t FootAngleDemand;
uint8_t FootDrive;
uint8_t FootParking;

uint32_t PackageLastTimeReset_Motherboard;
uint32_t PackageLastTimeReset_OnBoardPC;
uint32_t LastUpdateIMU;
uint32_t LastUpdateLogic;
uint32_t LastUpdateADC;
uint32_t LastUpdateServo;
uint32_t LastUpdateLed;

int ControlCommandTimeoutMS = 2000;

uint8_t InititionHall = 0;
float HallLeftStepPast = 0;
float HallRightStepPast = 0;
float HallLeftStep;
float HallRightStep;

uint8_t ParameterNumber;

uint8_t BalanceActiveDemand;
uint8_t BTBalanceActive;

uint8_t debug_driver_en = 0;
uint8_t debug_direction = 0;
uint32_t debug_steps = 10;
uint32_t debug_period = 1600;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UartLowPrepareRaw(uint16_t Difference, int32_t* InputHall, uint8_t Count);
int HallActualize(int32_t NewStep, int32_t LastStep, int32_t Difference);
void GPIOUpdate();
float Interpolation(float Value, float Min, float Max);
void ADCInit();
void ADCUpdate();
void ADCPrepare();
uint16_t ReadAdcChanel(uint8_t Channel);
void SerialLowControlLoop();
void ImuAccelUpdate();
void ImuGyroUpdate();
void ImuMagUpdate();
void ImuInit();
void ImuUpdate();
void StepControl(uint8_t dir, uint32_t period, uint32_t steps);
void MotopStop();
void BalancePrepare();
void BalanceCalculateSpeeds();
void BalancePositionLinearControl();
void BalanceSpeedLinearControl();
void BalancePositionAngularControl();
void BalanceLoop();
void BalanceResultLoop();
void DelayUs(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_StatusTypeDef Res;

	if (UartHandle->Instance == SYSTEM_HARDWARE_UART_HIGH_INSTANSE)
	{
		if (UartHighReceiveState == 0)
		{
			if ((SerialHighLevelRequest.CR != 13) || (SerialHighLevelRequest.LF != 10))
			{
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, LostByte, 1);
				UartHighReceiveState = 1;
			}
			else
			{
				UartHighReceiveState = 10;
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
			}
		}
		else
		{
			if(UartHighReceiveState == 1)
			{
				if (LostByte[0] == 13)
				{
					UartHighReceiveState = 2;
				}
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)LostByte, 1);
			}
			else
			{
				if (UartHighReceiveState == 2)
				{
					if (LostByte[0] == 10)
					{
						UartHighReceiveState = 0;
						Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
					}
					else
					{
						UartHighReceiveState = 1;
						Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)LostByte, 1);
					}
				}
			}
		}
		if (Res != HAL_OK)
		{
			MX_USART3_UART_Init();
			UartHighReceiveState = 0;
			Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
		}
	}

	if (UartHandle->Instance == SYSTEM_HARDWARE_UART_LOW_INSTANSE)
	{
		if (UartLowReceiveState == 0)
		{
			if ((SerialControlWheelsResponce.CR != 13) || (SerialControlWheelsResponce.LF != 10))
			{
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, LostByte, 1);
				UartLowReceiveState = 1;
			}
			else
			{
				UartLowReceiveState = 10;
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
			}
		}
		else
		{
			if(UartLowReceiveState == 1)
			{
				if (LostByte[0] == 13)
				{
					UartLowReceiveState = 2;
				}
				Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)LostByte, 1);
			}
			else
			{
				if (UartLowReceiveState == 2)
				{
					if (LostByte[0] == 10)
					{
						UartLowReceiveState = 0;
						Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
					}
					else
					{
						UartLowReceiveState = 1;
						Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)LostByte, 1);
					}
				}
			}
		}

		if (Res != HAL_OK)
		{
			MX_USART2_UART_Init();
			UartLowReceiveState = 0;
			Res = HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
		}
	}
}
void UartLowPrepareRaw(uint16_t Difference, int32_t* InputHall, uint8_t Count)
{
	if(InitionHall == 0)
	{
		for(int i = 0; i < Count; i++)
		{
			WheelsHall[i].LastHall = InputHall[i];
		}
		InitionHall = 1;
	}
	for (int i = 0; i < Count; i++)
	{
		if (HallActualize(InputHall[i], WheelsHall[i].LastHall, Difference))
		{
			WheelsHall[i].OutputHall = InputHall[i];
			WheelsHall[i].LastHall = InputHall[i];
		}
		if (!HallActualize(InputHall[i], WheelsHall[i].LastHall, Difference))
		{
			WheelsHall[i].LastHall = InputHall[i];
		}
	}

	switch (SerialControlWheelsResponce.ParameterNumber)
	{
		case 0:
			LowDiagnostic.Voltage = SerialControlWheelsResponce.ParameterValue;
			LowDiagnostic.Battery += ((Interpolation(LowDiagnostic.Voltage, 28, 41) * 100.0) - LowDiagnostic.Battery) * 0.01;
		  	break;
		case 1:
			LowDiagnostic.CurrentLeft = SerialControlWheelsResponce.ParameterValue;
			break;
		case 2:
			LowDiagnostic.CurrentRight = SerialControlWheelsResponce.ParameterValue;
			break;
		case 3:
			LowDiagnostic.RPSLeft = SerialControlWheelsResponce.ParameterValue;
			break;
		case 4:
			LowDiagnostic.RPSRight = SerialControlWheelsResponce.ParameterValue;
			break;
		case 5:
			LowDiagnostic.OverCurrCount = SerialControlWheelsResponce.ParameterValue;
			break;
		case 6:
			LowDiagnostic.ConnErrCount = SerialControlWheelsResponce.ParameterValue;
			break;
		case 7:
			LowDiagnostic.CommTime = SerialControlWheelsResponce.ParameterValue;
		  	break;
	}
}
int HallActualize(int32_t NewStep, int32_t LastStep, int32_t Difference)
{
	int32_t CalcDiff = abs(LastStep - NewStep);
	if (CalcDiff <= Difference)
	{
		return 1;
	}
	return 0;
}
void GPIOUpdate()
{
	FootButtonUp = HAL_GPIO_ReadPin(SYSTEM_HARDWARE_PARKING_LEG_UP_PORT, SYSTEM_HARDWARE_PARKING_LEG_UP_PIN);
	FootButtonDown = HAL_GPIO_ReadPin(SYSTEM_HARDWARE_PARKING_LEG_DOWN_PORT, SYSTEM_HARDWARE_PARKING_LEG_DOWN_PIN);
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
void ADCInit()
{
	uint16_t Raw[SYSTEM_HARDWARE_ADC_Channel_Count];
	uint16_t Sensors[SYSTEM_HARDWARE_ADC_Channel_Count - 3];
	float Amperage[SYSTEM_HARDWARE_ADC_Channel_Count - 5];
	AdcModule.Raw = Raw;
	AdcModule.Sensors = Sensors;
	AdcModule.Amperage = Amperage;
}
void ADCUpdate()
{
	for (int i = 0; i < SYSTEM_HARDWARE_ADC_Channel_Count; i++)
	{
		AdcModule.Raw[i] = ReadAdcChanel(i);
	}
#ifndef DEBUG_NO_ADC_ALL
#ifndef DEBUG_NO_ADC_RAW
	DebugADCRawFL = AdcModule.Raw[0];
	DebugADCRawFR = AdcModule.Raw[1];
	DebugADCRawBL = AdcModule.Raw[2];
	DebugADCRawBR = AdcModule.Raw[3];
	DebugADCRawCN = AdcModule.Raw[4];
	DebugADCRaw36 = AdcModule.Raw[5];
	DebugADCRaw12 = AdcModule.Raw[6];
	DebugADCRaw5 = AdcModule.Raw[7];
#endif
#endif
}
void ADCPrepare()
{
	for (int i = 0; i < SYSTEM_HARDWARE_ADC_Channel_Count - 3; i++) // Sensor
	{
		AdcModule.Sensors[i] = AdcModule.Raw[i] * 1; // No conversion
	}
	for (int i = SYSTEM_HARDWARE_ADC_Channel_Count - 3; i < SYSTEM_HARDWARE_ADC_Channel_Count - 2; i++) // Current 30A
	{
		AdcModule.Amperage[i - (SYSTEM_HARDWARE_ADC_Channel_Count - 3)] = (AdcModule.Raw[i] * 3.3 / 4095) * 0.066;
	}
	for (int i = SYSTEM_HARDWARE_ADC_Channel_Count - 2; i < SYSTEM_HARDWARE_ADC_Channel_Count; i++) // Current 20A
	{
		AdcModule.Amperage[i - (SYSTEM_HARDWARE_ADC_Channel_Count - 2)] = (AdcModule.Raw[i] * 3.3 / 4095) * 0.1;
	}
}
uint16_t ReadAdcChanel(uint8_t Channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	uint16_t RetVal;

	switch(Channel)
	{
	case 0:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_IK_FL;
		break;
	case 1:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_IK_FR;
		break;
	case 2:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_IK_BL;
		break;
	case 3:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_IK_BR;
		break;
	case 4:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_IK_CN;
		break;
	case 5:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_AMP_36;
		break;
	case 6:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_AMP_12;
		break;
	case 7:
		sConfig.Channel = SYSTEM_HARDWARE_ADC_AMP_5;
		break;
	}

	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(SYSTEM_HARDWARE_ADC, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(SYSTEM_HARDWARE_ADC);
	HAL_ADC_PollForConversion(SYSTEM_HARDWARE_ADC, 1000);
	RetVal = HAL_ADC_GetValue(SYSTEM_HARDWARE_ADC);
	HAL_ADC_Stop(SYSTEM_HARDWARE_ADC);
	return RetVal;
}
void SerialLowControlLoop()
{
	SerialControlWheelsRequest.ControlMode = 0;
	SerialControlWheelsRequest.ParameterNumber = 0;
	//SerialControlWheelsRequest.WheelLeft = BTControl.Front;
	//SerialControlWheelsRequest.WheelRight = BTControl.Turn;
	SerialControlWheelsRequest.CR=13;
	SerialControlWheelsRequest.LF=10;
	HAL_UART_Transmit_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)SerialControlWheelsRequest.Buffer, WHEELS_REQUEST_SIZE);
}
void ImuAccelUpdate()
{
	axises NewData;
	icm20948_accel_read(&NewData);

	ResAccel.x += roundf((((NewData.x / 16384) - ResAccel.x) * DebugFilterAcc) * DebugScallerAcc) / DebugScallerAcc;
	ResAccel.y += roundf((((NewData.y / 16384) - ResAccel.y) * DebugFilterAcc) * DebugScallerAcc) / DebugScallerAcc;
	ResAccel.z += roundf((((NewData.z / 16384) - ResAccel.z) * DebugFilterAcc) * DebugScallerAcc) / DebugScallerAcc;

	DebugAccRaw.x = NewData.x / 16384;
	DebugAccRaw.y = NewData.y / 16384;
	DebugAccRaw.z = NewData.z / 16384;

}
void ImuGyroUpdate()
{
	axises NewData;
	icm20948_gyro_read(&NewData);

	ResGyro.x += roundf((((NewData.x / 16.4) - ResGyro.x) * DebugFilterGyro) * DebugScallerGyro) / DebugScallerGyro;
	ResGyro.y += roundf((((NewData.y / 16.4) - ResGyro.y) * DebugFilterGyro) * DebugScallerGyro) / DebugScallerGyro;
	ResGyro.z += roundf((((NewData.z / 16.4) - ResGyro.z) * DebugFilterGyro) * DebugScallerGyro) / DebugScallerGyro;

	DebugGyroRaw.x = NewData.x / 16.4;
	DebugGyroRaw.y = NewData.y / 16.4;
	DebugGyroRaw.z = NewData.z / 16.4;
}
void ImuMagUpdate()
{
	axises NewData;
	ak09916_mag_read(&NewData);

	ResMag.x += roundf((((NewData.x * 0.15) - ResMag.x) * SYSTEM_IMU_MAG_FILTER) * 100) / 100;
	ResMag.y += roundf((((NewData.y * 0.15) - ResMag.y) * SYSTEM_IMU_MAG_FILTER) * 100) / 100;
	ResMag.z += roundf((((NewData.z * 0.15) - ResMag.z) * SYSTEM_IMU_MAG_FILTER) * 100) / 100;
}
void ImuInit()
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

	FusionBiasInitialise(&fusionBias, 0.4f, samplePeriod);
	FusionAhrsInitialise(&fusionAhrs, 0.6f);
}
void ImuUpdate()
{
	uncalibratedGyroscope.axis.x = ResGyro.x;
	uncalibratedGyroscope.axis.y = ResGyro.y;
	uncalibratedGyroscope.axis.z = ResGyro.z;

	uncalibratedAccelerometer.axis.x = ResAccel.x;
	uncalibratedAccelerometer.axis.y = ResAccel.y;
	uncalibratedAccelerometer.axis.z = ResAccel.z;

	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);
	FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);
	eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

	eulerAngles.angle.pitch = roundf(eulerAngles.angle.pitch * 1000) / 1000;
	eulerAngles.angle.roll = roundf(eulerAngles.angle.roll * 1000) / 1000;
	eulerAngles.angle.yaw = roundf(eulerAngles.angle.yaw * 1000) / 1000;
}
void StepControl(uint8_t dir, uint32_t period, uint32_t steps)
{
	HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PIN, 1);
	for(int i = 0; i <= steps; i++)
	{
		HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PIN, dir);
		HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PIN, i);
		//HAL_Delay(1);
		DelayUs(1);
		HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PIN, 0);
		//HAL_Delay(period);
		DelayUs(1000);
	}
	HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PIN, 0);
}
void MotopStop()
{
	HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PIN, 0);
	HAL_GPIO_WritePin(SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PORT, SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PIN, 0);
}
void BalancePrepare()
{
	Front = BTControl.Front;
	Turn = BTControl.Turn;

	if(LowDiagnostic.Battery)
	{
		Front = 0;
		Turn = 0;
	}

	if ((fabsf(Front) < 0.001) && (fabsf(Turn) < 0.001) && (fabsf(SpeedLinear) < 0.02) && (fabsf(LeftSpeed - RightSpeed) < 0.02))
	{
		if ((LowDiagnostic.Battery < 4) && BalanceActiveDemand)
		{
			BalanceActiveDemand = false;
		}
		else if (LowDiagnostic.Battery > 8)
		{
			BalanceActiveDemand = BTControl.Drive;
		}
		else if (!BTControl.Drive)
		{
			BalanceActiveDemand = false;
		}
	}

	Turn = (Turn > 90) ? 90 : Turn;
	Turn = (Turn < -90) ? -90 : Turn;
	Front = (Front > 2) ? 2 : Front;
	Front = (Front < -0.4) ? -0.4 : Front;

	if(!BalanceActiveDemand)
	{
		if(!FootButtonDown && !FootButtonUp)
		{
			StepControl(0, 1, 10);
			FootDone = false;
		} else if (FootButtonDown && !FootButtonUp)
		{
			MotopStop();
			FootDone = true;
		} else if (!FootButtonDown && FootButtonUp)
		{
			StepControl(0, 1, 10);
			FootDone = false;
		}
	} else if (BalanceActiveDemand)
	{
		if(!FootButtonDown && !FootButtonUp)
		{
			StepControl(1, 1, 10);
			FootDone = false;
		} else if (FootButtonDown && !FootButtonUp)
		{
			StepControl(1, 1, 10);
			FootDone = false;
		} else if (!FootButtonDown && FootButtonUp)
		{
			MotopStop();
			FootDone = true;
		}
	}

	if(FootDone)
	{
		BalanceActiveDemand = BTControl.Drive;
		BalanceActive = BTControl.Drive;
		//PositionLinearDemand = PositionLinear;
	}
	else
	{
		BalanceActiveDemand = false;
		BalanceActive = false;
		//PositionLinearDemand = PositionLinear;
	}
}
void BalanceCalculateSpeeds()
{
	TimeS = (HAL_GetTick() - StepsLeftPreviousTime) / 1000.0;
	if (TimeS > BALANCE_SPEED_STEPS_MAX_TIME)
	{
		TimeS = BALANCE_SPEED_STEPS_MAX_TIME;
		LeftSpeed = 0;
	}
	if (WheelsHall[0].OutputHall != StepsLeftPrevious)
	{
		LeftSpeed = ((WheelsHall[0].OutputHall - StepsLeftPrevious) * BALANCE_STEPS_TO_METERS) / TimeS;
		StepsLeftPrevious = WheelsHall[0].OutputHall;
		StepsLeftPreviousTime = HAL_GetTick();
	}

	TimeS = (HAL_GetTick() - StepsRightPreviousTime) / 1000.0;
	if (TimeS > BALANCE_SPEED_STEPS_MAX_TIME)
	{
		TimeS = BALANCE_SPEED_STEPS_MAX_TIME;
		RightSpeed = 0;
	}
	if (WheelsHall[1].OutputHall != StepsRightPrevious)
	{
		RightSpeed = ((WheelsHall[1].OutputHall - StepsRightPrevious) * BALANCE_STEPS_TO_METERS) / TimeS;
		StepsRightPrevious = WheelsHall[1].OutputHall;
		StepsRightPreviousTime = HAL_GetTick();
	}

	PositionLinear = ((WheelsHall[0].OutputHall + WheelsHall[1].OutputHall) / 2) * BALANCE_STEPS_TO_METERS;
	SpeedLinear = (LeftSpeed + RightSpeed) / 2.0;
}
void BalancePositionLinearControl()
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
void BalanceSpeedLinearControl()
{
	float deltaTime = (HAL_GetTick() - linearTime) / 1000000.0;
	linearTime = HAL_GetTick();

	if (deltaTime < 0)
	{
		deltaTime = linearDeltaTimePrev;
	}
	linearDeltaTimePrev = deltaTime;

	SpeedLinearDemand = SpeedLinearDemand > BALANCE_PLATFORM_Y_MAX ? BALANCE_PLATFORM_Y_MAX : SpeedLinearDemand;
	SpeedLinearDemand = SpeedLinearDemand < -BALANCE_PLATFORM_Y_MAX ? -BALANCE_PLATFORM_Y_MAX : SpeedLinearDemand;

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

		PlatformYDemand = (PlatformYDemand > BALANCE_PLATFORM_Y_MAX) ? BALANCE_PLATFORM_Y_MAX : PlatformYDemand;
		PlatformYDemand = (PlatformYDemand < -BALANCE_PLATFORM_Y_MAX) ? -BALANCE_PLATFORM_Y_MAX : PlatformYDemand;
	}
	else
	{
		SpeedLinearDemand = 0;
		linearIntegral = 0;
	}
}
void BalancePositionAngularControl()
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
	DutyTurn = (DutyTurn > BALANCE_DUTY_MAX_ANGULAR) ? BALANCE_DUTY_MAX_ANGULAR : DutyTurn;
	DutyTurn = (DutyTurn < -BALANCE_DUTY_MAX_ANGULAR) ? -BALANCE_DUTY_MAX_ANGULAR : DutyTurn;
}
void BalanceLoop()
{
	GyroY = eulerAngles.angle.pitch + PlatformYDemand + AngleCorrection;

	GyroYSpeed = GyroY - GyroYPrevious;
	GyroYPrevious = GyroY;

	if (BalanceActive)
	{
		BalancePID = -GyroY * (float)BalanceP - GyroYSpeed * (float)BalanceD;

		DutyFront += BalancePID * 0.000001;

		DutyFront = (DutyFront > BALANCE_DUTY_MAX_LINEAR) ? BALANCE_DUTY_MAX_LINEAR : DutyFront;
		DutyFront = (DutyFront < -BALANCE_DUTY_MAX_LINEAR) ? -BALANCE_DUTY_MAX_LINEAR : DutyFront;
	}
	else
	{
		PlatformYDemand = 0;
		DutyFront = 0;
	}
}
void BalanceResultLoop()
{
	ResultLeft += ((DutyFront + DutyTurn) - ResultLeft) * BalanceFilter;
	ResultRight += ((DutyFront - DutyTurn) - ResultRight) * BalanceFilter;

	SerialControlWheelsRequest.WheelLeft = ResultLeft;
	SerialControlWheelsRequest.WheelRight = ResultRight;

	SerialControlWheelsRequest.WheelLeft += (RotationI * (BTControl.Front));
	SerialControlWheelsRequest.WheelRight += (RotationI * (BTControl.Front));
	SerialControlWheelsRequest.WheelLeft += ManualDrive;
	SerialControlWheelsRequest.WheelRight += ManualDrive;

	if (eulerAngles.angle.pitch + AngleCorrection > 20)
	{
		SerialControlWheelsRequest.WheelLeft = 0;
		SerialControlWheelsRequest.WheelRight = 0;
	}
}
void DelayUs(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while (__HAL_TIM_GET_COUNTER(&htim8) < us);
}

//------FLUPDATE
uint8_t Fl_Update = 0;
//------MODE ANIMATIO
uint8_t Mode = 0;
//------POSITION LED
int8_t Pos = 0;
//------FLAG UP/DOWN FOR ANIMATION1
uint8_t Fl_Top = 0;
//-------FL USER BTN;
uint8_t FL_BTN = 0;
uint8_t Count_BTN=0;
//-----Animation2
uint8_t BRIGHTNESS=0;
uint8_t FL_BRIGHTNESS=0;
// color for brightness
uint8_t ColorRed=0;
uint8_t ColorGreen=0;
uint8_t ColorBlue=0;

//-----Animation3
uint8_t Pos1=0;
uint8_t Pos2=LED_COUNT-1;
uint8_t Fl_Top1 = 0;
uint8_t Fl_Top2 = 1;
void WS2812_PIXEL_RGB_TO_BUF_DMA(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX){
  for(uint8_t i=0;i<8;i++){
    if (BitIsSet(Rpixel,(7-i)) == 1){
      BUF_DMA[DELAY_LEN+posX*24+i+8] = HIGH;
    }else{
      BUF_DMA[DELAY_LEN+posX*24+i+8] = LOW;
    }
    if (BitIsSet(Gpixel,(7-i)) == 1){
      BUF_DMA[DELAY_LEN+posX*24+i+0] = HIGH;
    }else{
      BUF_DMA[DELAY_LEN+posX*24+i+0] = LOW;
    }
    if (BitIsSet(Bpixel,(7-i)) == 1){
      BUF_DMA[DELAY_LEN+posX*24+i+16] = HIGH;
    }else{
      BUF_DMA[DELAY_LEN+posX*24+i+16] = LOW;
    }
  }
}
void WS2812_LIGHT(void){
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&BUF_DMA, ARRAY_LEN);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)&BUF_DMA, ARRAY_LEN);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)&BUF_DMA, ARRAY_LEN);
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*)&BUF_DMA, ARRAY_LEN);
}
void WS2812_CLEAR(void){
	for (uint8_t i = 0; i < LED_COUNT; ++i){WS2812_PIXEL_RGB_TO_BUF_DMA(0, 0, 0, i);}
}
void WS2812_Init(uint8_t LedNumber){
	for (uint16_t i = DELAY_LEN; i < ARRAY_LEN; i++)BUF_DMA[i] = LOW;
	WS2812_CLEAR();
	WS2812_LIGHT();
	HAL_Delay(1);
}
//-------------Three LED up and down
void WS2812_ANIMATION_1(void){
	WS2812_CLEAR();
	WS2812_PIXEL_RGB_TO_BUF_DMA(MAX_LIGHT, 0, 0, Pos);
    if(Pos<(LED_COUNT-1)){WS2812_PIXEL_RGB_TO_BUF_DMA(0, 0, MAX_LIGHT, Pos+1);}
    if(Pos<(LED_COUNT-2)){WS2812_PIXEL_RGB_TO_BUF_DMA(0, MAX_LIGHT, 0, Pos+2);}
	WS2812_LIGHT();
	if(Fl_Top==0){
		Pos++;
		if(Pos==LED_COUNT-2){Fl_Top=1;Pos=LED_COUNT-4;}
	}else if (Fl_Top==1) {
		Pos--;
		if(Pos==0){Fl_Top=0;Pos=0;}
	}
}
void WS2812_ANIMATION_2(void) {
	for (uint8_t i = 0; i < LED_COUNT; i++) {
		WS2812_PIXEL_RGB_TO_BUF_DMA(ColorRed * BRIGHTNESS / 100,	ColorGreen * BRIGHTNESS / 100, ColorBlue * BRIGHTNESS / 100, i);
	}
	WS2812_LIGHT();
	if (FL_BRIGHTNESS == 0) {
		BRIGHTNESS++;
	if(BRIGHTNESS == MAX_BRIGHTNESS) {
		FL_BRIGHTNESS=1;
		BRIGHTNESS=MAX_BRIGHTNESS;
	}
} else if (FL_BRIGHTNESS == 1) {
	BRIGHTNESS--;
	if (BRIGHTNESS == 0) {
		FL_BRIGHTNESS = 0;
		BRIGHTNESS = 0;
		ColorRed = rand()%255;
		ColorGreen = rand()%255;
		ColorBlue = rand()%255;
	}
}
}

void WS2812_UPDATE(void){
	if (FL_BTN==1) {
		FL_BTN=0;
		Mode++;
		if(Mode>2)Mode=0;
		Pos = 0;
		Fl_Top = 0;
		BRIGHTNESS=0;
		FL_BRIGHTNESS=0;
		ColorRed=128;
		ColorGreen=70;
		ColorBlue=30;
		Pos1=0;
		Pos2=LED_COUNT-1;
		Fl_Top1 = 0;
		Fl_Top2 = 1;
	}
	switch (Mode) {
	case 0:
		WS2812_ANIMATION_1();
		break;
	case 1:
		WS2812_ANIMATION_2();
		break;
	}
	Fl_Update = 0;
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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
#ifndef SYSTEM_NO_ADC_INIT
  ADCInit();
#endif
#ifndef SYSTEM_NO_IMU_HW_INIT
  icm20948_init();
  ak09916_init();
#endif
#ifndef SYSTEM_NO_IMU_INIT
  ImuInit();
#endif

#ifndef SYSTEM_NO_LED_INIT
  WS2812_Init(0);
  WS2812_Init(1);
  WS2812_Init(2);
  WS2812_Init(3);
  ColorRed = rand() % 255;
  ColorGreen = rand() % 255;
  ColorBlue = rand() % 255;
#endif
  HAL_TIM_Base_Start(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifndef SYSTEM_NO_LOW_UART_LOOP
	  if (HAL_GetTick() - LastPkgTimeUartLow > SYSTEM_TIMING_MS_UART_LOW)
	  {
		  MX_USART2_UART_Init();
		  UartLowReceiveState = 0;
		  HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_LOW, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
		  LastPkgTimeUartLow = HAL_GetTick();
	  }
#endif
#ifndef SYSTEM_NO_HIGH_UART_LOOP
	  if (HAL_GetTick() - LastPkgTimeUartHigh > SYSTEM_TIMING_MS_UART_HIGH)
	  {
		  MX_USART3_UART_Init();
		  UartHighReceiveState = 0;
		  HAL_UART_Receive_DMA(SYSTEM_HARDWARE_UART_HIGH, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
		  LastPkgTimeUartHigh = HAL_GetTick();
	  }
#endif
#ifndef SYSTEM_NO_LOW_UART_LOOP
	  if ((UartLowReceiveState == 10) && (SerialControlWheelsResponce.CR == 13) && (SerialControlWheelsResponce.LF == 10))
	  {
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		  UartLowReceiveState = 0;
		  int32_t TemplateWheels[2] = { (SerialControlWheelsResponce.WheelLeftSteps * -1), (SerialControlWheelsResponce.WheelRightSteps * -1) };
		  UartLowPrepareRaw(SYSTEM_HALL_FILTER_MAX, TemplateWheels, 2);
		  LastPkgTimeUartLow = HAL_GetTick();
	  }
#endif
#ifndef SYSTEM_NO_GPIO_LOOP
	  if (HAL_GetTick() - LastUpdateGPIO > SYSTEM_TIMING_MS_GPIO)
	  {
		  GPIOUpdate();
		  LastUpdateGPIO = HAL_GetTick();
	  }
#endif
#ifndef SYSTEM_NO_ADC_INIT
#ifndef SYSTEM_NO_ADC_LOOP
	  if (HAL_GetTick() - LastUpdateADC > SYSTEM_TIMING_MS_ADC)
	  {
		  ADCUpdate();
		  ADCPrepare();
		  LastUpdateADC = HAL_GetTick();
	  }
#endif
#endif

	  if(HAL_GetTick() - LastUpdateIMU > SYSTEM_TIMING_MS_IMU)
	  {
		  ImuAccelUpdate();
		  ImuGyroUpdate();
		  ImuMagUpdate();
		  ImuUpdate();
		  LastUpdateIMU = HAL_GetTick();
	  }

#ifndef SYSTEM_NO_HIGH_UART_LOOP
	  if ((UartHighReceiveState == 10) && (SerialHighLevelRequest.CR == 13) && (SerialHighLevelRequest.LF == 10))
	  {
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		  BTControl.Front = SerialHighLevelRequest.Linear;
		  BTControl.Turn = SerialHighLevelRequest.Angular;
		  BTControl.Drive = SerialHighLevelRequest.DriveMode;

		  UartHighReceiveState = 0;
		  SerialHighLevelResponce.ControllerState = -1;
		  SerialHighLevelResponce.WheelLeftSteps = WheelsHall[0].OutputHall;
		  SerialHighLevelResponce.WheelRightSteps = WheelsHall[1].OutputHall;
		  SerialHighLevelResponce.BatteryPersentage = LowDiagnostic.Battery;
		  SerialHighLevelResponce.Roll = eulerAngles.angle.roll;
		  SerialHighLevelResponce.Pitch = eulerAngles.angle.pitch;
		  SerialHighLevelResponce.Yaw = eulerAngles.angle.yaw;
		  SerialHighLevelResponce.CenterIkSensor = AdcModule.Sensors[4];
		  SerialHighLevelResponce.ParameterNumber = 0;
		  SerialHighLevelResponce.ParametrValue = 0;
		  SerialHighLevelResponce.CR = 13;
		  SerialHighLevelResponce.LF = 10;
 		  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)SerialHighLevelResponce.Buffer, HIGH_LEVEL_RESPONCE_SIZE);
		  LastPkgTimeUartHigh = HAL_GetTick();
	  }
#endif

	  	  if (HAL_GetTick() - LastUpdateLed > 50)
	  	  {
	  		  WS2812_UPDATE();
	  		  LastUpdateLed = HAL_GetTick();
	  	  }

	  if (HAL_GetTick() - LastUpdateLogic > SYSTEM_TIMING_MS_LOGIC)
	  {
		  BalancePrepare();						// CMPLT
		  BalanceCalculateSpeeds();				// CMPLT
		  BalancePositionLinearControl();		//
		  //BalanceSpeedLinearControl();			//
		  //BalancePositionAngularControl();
		  //BalanceLoop();						//
		  //BalanceResultLoop();					//
		  LastUpdateLogic = HAL_GetTick();
	  }

#ifndef SYSTEM_NO_LOW_UART_LOOP
	  SerialLowControlLoop();
#endif
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
