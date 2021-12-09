################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ICM20948/icm20948.c 

C_DEPS += \
./Core/ICM20948/icm20948.d 

OBJS += \
./Core/ICM20948/icm20948.o 


# Each subdirectory must supply rules for building sources it contributes
Core/ICM20948/%.o: ../Core/ICM20948/%.c Core/ICM20948/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Lord Tachanka/STM32CubeIDE/workspace_1.7.0/stm_gyro_stable_controller/Core/FusionIMU" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Lord Tachanka/STM32CubeIDE/workspace_1.7.0/stm_gyro_stable_controller/Core/ICM20948" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

