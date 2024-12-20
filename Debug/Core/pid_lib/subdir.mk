################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/pid_lib/PID.c 

OBJS += \
./Core/pid_lib/PID.o 

C_DEPS += \
./Core/pid_lib/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/pid_lib/%.o Core/pid_lib/%.su Core/pid_lib/%.cyclo: ../Core/pid_lib/%.c Core/pid_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H503xx -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-pid_lib

clean-Core-2f-pid_lib:
	-$(RM) ./Core/pid_lib/PID.cyclo ./Core/pid_lib/PID.d ./Core/pid_lib/PID.o ./Core/pid_lib/PID.su

.PHONY: clean-Core-2f-pid_lib

