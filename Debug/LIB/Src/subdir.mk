################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LIB/Src/74HC595.c \
../LIB/Src/Flag.c \
../LIB/Src/LCD.c \
../LIB/Src/PMS7003.c \
../LIB/Src/SHT31.c 

OBJS += \
./LIB/Src/74HC595.o \
./LIB/Src/Flag.o \
./LIB/Src/LCD.o \
./LIB/Src/PMS7003.o \
./LIB/Src/SHT31.o 

C_DEPS += \
./LIB/Src/74HC595.d \
./LIB/Src/Flag.d \
./LIB/Src/LCD.d \
./LIB/Src/PMS7003.d \
./LIB/Src/SHT31.d 


# Each subdirectory must supply rules for building sources it contributes
LIB/Src/%.o LIB/Src/%.su: ../LIB/Src/%.c LIB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/DU_AN_IOT/firmware/DATN/DATN/LIB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LIB-2f-Src

clean-LIB-2f-Src:
	-$(RM) ./LIB/Src/74HC595.d ./LIB/Src/74HC595.o ./LIB/Src/74HC595.su ./LIB/Src/Flag.d ./LIB/Src/Flag.o ./LIB/Src/Flag.su ./LIB/Src/LCD.d ./LIB/Src/LCD.o ./LIB/Src/LCD.su ./LIB/Src/PMS7003.d ./LIB/Src/PMS7003.o ./LIB/Src/PMS7003.su ./LIB/Src/SHT31.d ./LIB/Src/SHT31.o ./LIB/Src/SHT31.su

.PHONY: clean-LIB-2f-Src

