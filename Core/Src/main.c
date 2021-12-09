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

#define HIGH_LEVEL_REQUEST_SIZE 14				// Размер пакета
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTEM_NO_GYRO_INIT
#define SYSTEM_NO_IMU_INIT
#define SYSTEM_NO_PARK_INIT
#define SYSTEM_NO_LED_INIT

#define SYSTEM_HARDWARE_UART0 (&huart2)
#define SYSTEM_HARDWARE_UART1 (&huart3)
#define SYSTEM_HARDWARE_ADC (&hadc1)
#define SYSTEM_HARDWARE_ADC_IK_FL ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_IK_FR ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_IK_BL ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_IK_BR ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_IK_CN ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_AMP_36 ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_AMP_12 ADC_CHANNEL_0
#define SYSTEM_HARDWARE_ADC_AMP_5 ADC_CHANNEL_0
#define SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PORT GPIOC
#define SYSTEM_HARDWARE_STEPPER_MOTOR_STEP_PIN GPIO_PIN_4
#define SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PORT GPIOC
#define SYSTEM_HARDWARE_STEPPER_MOTOR_DIR_PIN GPIO_PIN_5
#define SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PORT GPIOE
#define SYSTEM_HARDWARE_STEPPER_MOTOR_EN_PIN GPIO_PIN_8
#define SYSTEM_HARDWARE_PARKING_LEG_UP_PORT GPIOE
#define SYSTEM_HARDWARE_PARKING_LEG_UP_PIN GPIO_PIN_4
#define SYSTEM_HARDWARE_PARKING_LEG_DOWN_PORT GPIOE
#define SYSTEM_HARDWARE_PARKING_LEG_DOWN_PIN GPIO_PIN_5
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

//#define DELAY_LEN 48
#define LED_COUNT 16
#define HIGH 65
#define LOW 26
#define ARRAY_LEN DELAY_LEN+LED_COUNT*24
#define BitIsSet(reg, bit) ((reg & (1<<bit))!=0)
#define MAX_BRIGHTNESS 50
uint32_t BUF_DMA[ARRAY_LEN]={0};

uint16_t MAX_LIGHT = 128;


#define MOTHERBOARD_DIFF 100

#define PARKING_SPEED_DRIVE 15
#define PARKING_SPEED_PARKING 20;

#define PARKING_MAX 1000

#define SPEED_STEPS_MAX_TIME 0.2
#define STEPS_TO_METERS 0.0088495575221239

#define PLATFORM_Y_MAX 5
#define DUTY_MAX_ANGULAR 0.1
#define DUTY_MAX_LINEAR 0.15

#define ADC_CH_COUNT 8
#define STEPPER_DRIVE_MAX 3200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t UART1ReceiveState=0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good  //OnBoard Plate
volatile uint8_t UART2ReceiveState=0; // 0 - by default; 1 - trouble by CR/LF; 10 - pkg good  //STM Plate

//global for debug
float TimeS;
//----------------

LowUartData STM32Gyroscooter;

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

float BalanceP = 600;
float BalanceD = 15000;

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

uint32_t CurrentStepPark = 0;

uint8_t PositionLinearControlSwitch;
uint8_t BalanceActive;
int16_t ParkingPersentage;
uint8_t FootAngleDemand;
uint8_t FootDrive;
uint8_t FootParking;

uint8_t* LostByte;

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
float samplePeriod = 0.01f;
FusionVector3 gyroscopeSensitivity;
FusionVector3 accelerometerSensitivity;
FusionVector3 hardIronBias;
FusionVector3 uncalibratedGyroscope;
FusionVector3 uncalibratedAccelerometer;
FusionVector3 uncalibratedMagnetometer;
FusionEulerAngles eulerAngles;

uint16_t ADC_VAL[ADC_CH_COUNT] = {0,};

uint8_t BTN_PARK_UP;
uint8_t BTN_PARK_DOWN;

uint16_t dADC0;
uint16_t dADC1;
uint16_t dADC2;
uint16_t dADC3;
uint16_t dADC4;
uint16_t dADC5;
uint16_t dADC6;
uint16_t dADC7;
uint16_t dADC8;
uint16_t dADC9;

uint8_t debug_driver_en = 0;
uint8_t debug_direction = 0;
uint32_t debug_steps = 10;
uint32_t debug_period = 1600;

uint8_t debug_led_en = 0;
uint8_t debug_led_mode = 0;

uint8_t debug_ADC_0 = 0;
uint8_t debug_ADC_1 = 0;
uint8_t debug_ADC_2 = 0;
uint8_t debug_ADC_3 = 0;
uint8_t debug_ADC_4 = 0;
uint8_t debug_ADC_5 = 0;
uint8_t debug_ADC_6 = 0;
uint8_t debug_ADC_7 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Select_CH(uint8_t ChanelNum);
void ADC_Update();
void DrivePrepare();
void DriveToStep(uint8_t dir, uint32_t period, uint32_t steps);
void DriveStop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_StatusTypeDef Res;

	if (UartHandle->Instance == USART3){ // Jetson commutation
		if (USART1ReceiveState == 0){
			if ((SerialHighLevelRequest.CR != 13) || (SerialHighLevelRequest.LF != 10)){
				Res = HAL_UART_Receive_DMA(&huart3, LostByte, 1);
				USART1ReceiveState = 1;
			}
			else{
 				USART1ReceiveState = 10;
				Res = HAL_UART_Receive_DMA(&huart3, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
			}
		}
		else{
			if(USART1ReceiveState == 1){
				if (LostByte[0] == 13){
					USART1ReceiveState = 2;
				}
				Res = HAL_UART_Receive_DMA(&huart3, (uint8_t*)LostByte, 1);
			}
			else{
				if (USART1ReceiveState == 2){
					if (LostByte[0] == 10){
						USART1ReceiveState = 0;
						Res = HAL_UART_Receive_DMA(&huart3, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
					}
					else{
						USART1ReceiveState = 1;
						Res = HAL_UART_Receive_DMA(&huart3, (uint8_t*)LostByte, 1);
					}
				}
			}
		}
		if (Res != HAL_OK)
		{
			MX_USART3_UART_Init();
			USART1ReceiveState = 0;
			Res = HAL_UART_Receive_DMA(&huart3, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
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

	icm20948_accel_read(&my_accel);
	icm20948_gyro_read(&my_gyro);

	uncalibratedGyroscope.axis.x = my_gyro.x;
	uncalibratedGyroscope.axis.y = my_gyro.y;
	uncalibratedGyroscope.axis.z = my_gyro.z;

	uncalibratedAccelerometer.axis.x = my_accel.x;
	uncalibratedAccelerometer.axis.y = my_accel.y;
	uncalibratedAccelerometer.axis.z = my_accel.z;

	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);
	FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);
	eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
}
void SERIAL_CONTROL_LOOP()
{
	SerialControlWheelsRequest.ControlMode = 0;
	SerialControlWheelsRequest.ParameterNumber = 0;
	SerialControlWheelsRequest.WheelLeft = BTFront;
	SerialControlWheelsRequest.WheelRight = BTTurn;
	SerialControlWheelsRequest.CR=13;
	SerialControlWheelsRequest.LF=10;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)SerialControlWheelsRequest.Buffer, WHEELS_REQUEST_SIZE);
}

void SERIAL_ONBOARD_LOOP()
{
	SerialOnBoardRequest.Linear = 0;
	SerialOnBoardRequest.Angular = 0;
	SerialOnBoardRequest.CR = 13;
	SerialOnBoardRequest.LF = 10;
	//HAL_UART_Transmit_DMA(&huart6, (uint8_t*)SerialOnBoardRequest.Buffer, ON_BOARD_CONTROL_REQUEST_SIZE);
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

	if (BalanceActiveDemand)
	{

	}
	else
	{
		FootAngleDemand = STEPPER_DRIVE_MAX;
	}

	//BalanceActiveDemand = BTBalanceActive;
	//BalanceActive = BTBalanceActive;
	//PositionLinearDemand = PositionLinear;
}
void BALANCE_Calculate_Speeds()
{
	//LEFT
	TimeS = (HAL_GetTick() - StepsLeftPreviousTime) / 1000.0;
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
	GyroY = (eulerAngles.angle.pitch * 1) + PlatformYDemand + AngleCorrection - ParkingAngle;

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
void ADC_Select_CH(uint8_t ChanelNum)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	switch(ChanelNum)
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_8;
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_9;
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_10;
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_11;
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_12;
		break;
	}

	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[ChanelNum] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

}

void ADC_Update()
{
	for (int i = 0; i < ADC_CH_COUNT; i++)
	{
		ADC_Select_CH(i);
	}

	dADC0 = ADC_VAL[0];
	dADC1 = ADC_VAL[1];
	dADC2 = ADC_VAL[2];
	dADC3 = ADC_VAL[3];
	dADC4 = ADC_VAL[4];
	dADC5 = ADC_VAL[5];
	dADC6 = ADC_VAL[6];
	dADC7 = ADC_VAL[7];

}
void StepControl(uint8_t dir, uint32_t period, uint32_t steps)
{
	for(int i = 0; i <= steps; i++)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, dir);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, i);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_Delay(period);
	}
}

void MotopStop()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
}
void DrivePrepare()
{
	StepControl(0,1,STEPPER_DRIVE_MAX);
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
  /* USER CODE BEGIN 2 */
#ifndef SYSTEM_NO_GYRO_INIT
  icm20948_init();
#endif
#ifndef SYSTEM_NO_IMU_INIT
  IMU_INIT();
#endif
#ifndef SYSTEM_NO_PARK_INIT
  DrivePrepare();
#endif
#ifndef SYSTEM_NO_LED_INIT
  WS2812_Init();
  ColorRed = rand() % 255;
  ColorGreen = rand() % 255;
  ColorBlue = rand() % 255;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BTN_PARK_UP = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
	  BTN_PARK_DOWN = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);

	  if (HAL_GetTick() - LastUpdateLed > 100)
	  {
		  WS2812_UPDATE();
		  LastUpdateLed = HAL_GetTick();
	  }

	  if (HAL_GetTick() - LastUpdateIMU > 1)
	  {
		  IMU_UPDATE();
		  LastUpdateIMU = HAL_GetTick();
	  }

	  if (HAL_GetTick() - LastUpdateADC > 10)
	  {
		  ADC_Update();
		  LastUpdateADC = HAL_GetTick();
	  }

	  if (HAL_GetTick() - PackageLastTimeReset_Motherboard > 100)
	  {
		  MX_USART2_UART_Init();
		  USART2ReceiveState = 0;
		  HAL_UART_Receive_DMA(&huart2, (uint8_t*)SerialControlWheelsResponce.Buffer, WHEELS_RESPONCE_SIZE);
		  PackageLastTimeReset_Motherboard = HAL_GetTick();
	  }

	  if (HAL_GetTick() - PackageLastTimeReset_OnBoardPC > 100)
	  {
		  MX_USART3_UART_Init();
		  USART1ReceiveState = 0;
		  HAL_UART_Receive_DMA(&huart3, (uint8_t*)SerialHighLevelRequest.Buffer, HIGH_LEVEL_REQUEST_SIZE);
		  PackageLastTimeReset_OnBoardPC = HAL_GetTick();
	  }

	  if ((USART2ReceiveState == 10) && (SerialControlWheelsResponce.CR == 13) && (SerialControlWheelsResponce.LF == 10))
	  {
		  USART2ReceiveState = 0;
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

		  if(InititionHall == 0)
		  {
			  HallLeftStepPast = SerialControlWheelsResponce.WheelLeftSteps;
			  HallRightStepPast = SerialControlWheelsResponce.WheelRightSteps;
			  InititionHall = 1;
		  }

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

	  if ((USART1ReceiveState == 10) && (SerialHighLevelRequest.CR == 13) && (SerialHighLevelRequest.LF == 10))
	  {
		  USART1ReceiveState = 0;
		  SerialHighLevelResponce.ControllerState = 0;
		  SerialHighLevelResponce.WheelLeftSteps = 0;
		  SerialHighLevelResponce.WheelRightSteps = 0;
		  SerialHighLevelResponce.BatteryPersentage = 0;
		  SerialHighLevelResponce.Roll = 0;
		  SerialHighLevelResponce.Pitch = 0;
		  SerialHighLevelResponce.Yaw = 0;
		  SerialHighLevelResponce.CenterIkSensor = 0;
		  SerialHighLevelResponce.ParameterNumber = 0;
		  SerialHighLevelResponce.ParametrValue = 0;
		  SerialHighLevelResponce.CR = 13;
		  SerialHighLevelResponce.LF = 10;
 		  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)SerialHighLevelResponce.Buffer, HIGH_LEVEL_RESPONCE_SIZE);
		  PackageLastTimeReset_OnBoardPC = HAL_GetTick();
	  }

	  if (HAL_GetTick() - LastUpdateLogic > 10)

	  {
		  BALANCE_Prepare();
		  BALANCE_Calculate_Speeds();
		  BALANCE_Position_Linear_Control();
		  BALANCE_Speed_LinearControl();
		  BALANCE_Position_Angular_Control();
		  BALANCE_LOOP();
		  BALANCE_Result_Loop();
		  LastUpdateLogic = HAL_GetTick();
	  }

	  /*if(debug_driver_en)
	  {
		  StepControl(debug_direction, debug_period, debug_steps);
		  debug_driver_en = 0;
	  }
	  else
	  {
		  MotopStop();
	  }*/

	  SERIAL_CONTROL_LOOP();
	  //SERIAL_ONBOARD_LOOP();

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
