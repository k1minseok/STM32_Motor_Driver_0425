################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/Button/Button.c 

OBJS += \
./Core/Driver/Button/Button.o 

C_DEPS += \
./Core/Driver/Button/Button.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/Button/%.o Core/Driver/Button/%.su Core/Driver/Button/%.cyclo: ../Core/Driver/Button/%.c Core/Driver/Button/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Driver-2f-Button

clean-Core-2f-Driver-2f-Button:
	-$(RM) ./Core/Driver/Button/Button.cyclo ./Core/Driver/Button/Button.d ./Core/Driver/Button/Button.o ./Core/Driver/Button/Button.su

.PHONY: clean-Core-2f-Driver-2f-Button

