################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/oled.c \
../User/task.c 

OBJS += \
./User/oled.o \
./User/task.o 

C_DEPS += \
./User/oled.d \
./User/task.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o User/%.su: ../User/%.c User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Administrator/STM32CubeIDE/workspace_1.6.1/uRCL/Include" -I"C:/Users/39475/OneDrive/Code/uRCL/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User

clean-User:
	-$(RM) ./User/oled.d ./User/oled.o ./User/oled.su ./User/task.d ./User/task.o ./User/task.su

.PHONY: clean-User

