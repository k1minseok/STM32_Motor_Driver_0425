################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/Motor/Motor.c 

OBJS += \
./Core/Driver/Motor/Motor.o 

C_DEPS += \
./Core/Driver/Motor/Motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/Motor/%.o Core/Driver/Motor/%.su Core/Driver/Motor/%.cyclo: ../Core/Driver/Motor/%.c Core/Driver/Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Driver-2f-Motor

clean-Core-2f-Driver-2f-Motor:
	-$(RM) ./Core/Driver/Motor/Motor.cyclo ./Core/Driver/Motor/Motor.d ./Core/Driver/Motor/Motor.o ./Core/Driver/Motor/Motor.su

.PHONY: clean-Core-2f-Driver-2f-Motor

