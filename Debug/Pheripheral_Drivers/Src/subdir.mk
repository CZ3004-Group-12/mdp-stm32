################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Pheripheral_Drivers/Src/diff.c \
../Pheripheral_Drivers/Src/dist.c \
../Pheripheral_Drivers/Src/encoder.c \
../Pheripheral_Drivers/Src/icm20948.c \
../Pheripheral_Drivers/Src/motor.c \
../Pheripheral_Drivers/Src/oled.c \
../Pheripheral_Drivers/Src/pid.c \
../Pheripheral_Drivers/Src/speed.c \
../Pheripheral_Drivers/Src/uart.c \
../Pheripheral_Drivers/Src/vehicleservo.c 

OBJS += \
./Pheripheral_Drivers/Src/diff.o \
./Pheripheral_Drivers/Src/dist.o \
./Pheripheral_Drivers/Src/encoder.o \
./Pheripheral_Drivers/Src/icm20948.o \
./Pheripheral_Drivers/Src/motor.o \
./Pheripheral_Drivers/Src/oled.o \
./Pheripheral_Drivers/Src/pid.o \
./Pheripheral_Drivers/Src/speed.o \
./Pheripheral_Drivers/Src/uart.o \
./Pheripheral_Drivers/Src/vehicleservo.o 

C_DEPS += \
./Pheripheral_Drivers/Src/diff.d \
./Pheripheral_Drivers/Src/dist.d \
./Pheripheral_Drivers/Src/encoder.d \
./Pheripheral_Drivers/Src/icm20948.d \
./Pheripheral_Drivers/Src/motor.d \
./Pheripheral_Drivers/Src/oled.d \
./Pheripheral_Drivers/Src/pid.d \
./Pheripheral_Drivers/Src/speed.d \
./Pheripheral_Drivers/Src/uart.d \
./Pheripheral_Drivers/Src/vehicleservo.d 


# Each subdirectory must supply rules for building sources it contributes
Pheripheral_Drivers/Src/%.o: ../Pheripheral_Drivers/Src/%.c Pheripheral_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Pheripheral_Drivers/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Pheripheral_Drivers-2f-Src

clean-Pheripheral_Drivers-2f-Src:
	-$(RM) ./Pheripheral_Drivers/Src/diff.d ./Pheripheral_Drivers/Src/diff.o ./Pheripheral_Drivers/Src/dist.d ./Pheripheral_Drivers/Src/dist.o ./Pheripheral_Drivers/Src/encoder.d ./Pheripheral_Drivers/Src/encoder.o ./Pheripheral_Drivers/Src/icm20948.d ./Pheripheral_Drivers/Src/icm20948.o ./Pheripheral_Drivers/Src/motor.d ./Pheripheral_Drivers/Src/motor.o ./Pheripheral_Drivers/Src/oled.d ./Pheripheral_Drivers/Src/oled.o ./Pheripheral_Drivers/Src/pid.d ./Pheripheral_Drivers/Src/pid.o ./Pheripheral_Drivers/Src/speed.d ./Pheripheral_Drivers/Src/speed.o ./Pheripheral_Drivers/Src/uart.d ./Pheripheral_Drivers/Src/uart.o ./Pheripheral_Drivers/Src/vehicleservo.d ./Pheripheral_Drivers/Src/vehicleservo.o

.PHONY: clean-Pheripheral_Drivers-2f-Src

