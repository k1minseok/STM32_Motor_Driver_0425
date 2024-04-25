################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/LED/LED.c 

OBJS += \
./Core/Driver/LED/LED.o 

C_DEPS += \
./Core/Driver/LED/LED.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/LED/%.o Core/Driver/LED/%.su Core/Driver/LED/%.cyclo: ../Core/Driver/LED/%.c Core/Driver/LED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Driver-2f-LED

clean-Core-2f-Driver-2f-LED:
	-$(RM) ./Core/Driver/LED/LED.cyclo ./Core/Driver/LED/LED.d ./Core/Driver/LED/LED.o ./Core/Driver/LED/LED.su

.PHONY: clean-Core-2f-Driver-2f-LED

