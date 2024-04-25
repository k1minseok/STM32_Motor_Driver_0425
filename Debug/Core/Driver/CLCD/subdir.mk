################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/CLCD/CLCD.c 

OBJS += \
./Core/Driver/CLCD/CLCD.o 

C_DEPS += \
./Core/Driver/CLCD/CLCD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/CLCD/%.o Core/Driver/CLCD/%.su Core/Driver/CLCD/%.cyclo: ../Core/Driver/CLCD/%.c Core/Driver/CLCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Driver-2f-CLCD

clean-Core-2f-Driver-2f-CLCD:
	-$(RM) ./Core/Driver/CLCD/CLCD.cyclo ./Core/Driver/CLCD/CLCD.d ./Core/Driver/CLCD/CLCD.o ./Core/Driver/CLCD/CLCD.su

.PHONY: clean-Core-2f-Driver-2f-CLCD

