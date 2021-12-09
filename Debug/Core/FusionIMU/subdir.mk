################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FusionIMU/FusionAhrs.c \
../Core/FusionIMU/FusionBias.c \
../Core/FusionIMU/FusionCompass.c 

C_DEPS += \
./Core/FusionIMU/FusionAhrs.d \
./Core/FusionIMU/FusionBias.d \
./Core/FusionIMU/FusionCompass.d 

OBJS += \
./Core/FusionIMU/FusionAhrs.o \
./Core/FusionIMU/FusionBias.o \
./Core/FusionIMU/FusionCompass.o 


# Each subdirectory must supply rules for building sources it contributes
Core/FusionIMU/%.o: ../Core/FusionIMU/%.c Core/FusionIMU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Lord Tachanka/STM32CubeIDE/workspace_1.7.0/stm_gyro_stable_controller/Core/FusionIMU" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Lord Tachanka/STM32CubeIDE/workspace_1.7.0/stm_gyro_stable_controller/Core/ICM20948" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

