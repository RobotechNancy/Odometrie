################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0X/Platform/TOF.c \
../Drivers/VL53L0X/Platform/vl53l0x_platform.c 

OBJS += \
./Drivers/VL53L0X/Platform/TOF.o \
./Drivers/VL53L0X/Platform/vl53l0x_platform.o 

C_DEPS += \
./Drivers/VL53L0X/Platform/TOF.d \
./Drivers/VL53L0X/Platform/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/Platform/%.o Drivers/VL53L0X/Platform/%.su: ../Drivers/VL53L0X/Platform/%.c Drivers/VL53L0X/Platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"C:/Users/yafong/robotech/Communication/CAN/Raspberry/include" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-Platform

clean-Drivers-2f-VL53L0X-2f-Platform:
	-$(RM) ./Drivers/VL53L0X/Platform/TOF.d ./Drivers/VL53L0X/Platform/TOF.o ./Drivers/VL53L0X/Platform/TOF.su ./Drivers/VL53L0X/Platform/vl53l0x_platform.d ./Drivers/VL53L0X/Platform/vl53l0x_platform.o ./Drivers/VL53L0X/Platform/vl53l0x_platform.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-Platform

