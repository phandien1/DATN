################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LIB/PMS7003/PMS7003.c 

OBJS += \
./LIB/PMS7003/PMS7003.o 

C_DEPS += \
./LIB/PMS7003/PMS7003.d 


# Each subdirectory must supply rules for building sources it contributes
LIB/PMS7003/%.o LIB/PMS7003/%.su: ../LIB/PMS7003/%.c LIB/PMS7003/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/DU_AN_IOT/firmware/DATN/DATN/LIB" -I"D:/DU_AN_IOT/firmware/DATN/DATN/LIB/PMS7003" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LIB-2f-PMS7003

clean-LIB-2f-PMS7003:
	-$(RM) ./LIB/PMS7003/PMS7003.d ./LIB/PMS7003/PMS7003.o ./LIB/PMS7003/PMS7003.su

.PHONY: clean-LIB-2f-PMS7003

