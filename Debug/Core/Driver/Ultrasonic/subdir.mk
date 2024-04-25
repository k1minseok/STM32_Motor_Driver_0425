################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/Ultrasonic/Ultrasonic.c 

OBJS += \
./Core/Driver/Ultrasonic/Ultrasonic.o 

C_DEPS += \
./Core/Driver/Ultrasonic/Ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/Ultrasonic/%.o Core/Driver/Ultrasonic/%.su Core/Driver/Ultrasonic/%.cyclo: ../Core/Driver/Ultrasonic/%.c Core/Driver/Ultrasonic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Driver-2f-Ultrasonic

clean-Core-2f-Driver-2f-Ultrasonic:
	-$(RM) ./Core/Driver/Ultrasonic/Ultrasonic.cyclo ./Core/Driver/Ultrasonic/Ultrasonic.d ./Core/Driver/Ultrasonic/Ultrasonic.o ./Core/Driver/Ultrasonic/Ultrasonic.su

.PHONY: clean-Core-2f-Driver-2f-Ultrasonic

