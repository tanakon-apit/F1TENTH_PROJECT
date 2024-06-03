################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/src/AS5600.c \
../Library/src/BNO055.c \
../Library/src/Controller.c \
../Library/src/RC.c \
../Library/src/Tamiya.c 

OBJS += \
./Library/src/AS5600.o \
./Library/src/BNO055.o \
./Library/src/Controller.o \
./Library/src/RC.o \
./Library/src/Tamiya.o 

C_DEPS += \
./Library/src/AS5600.d \
./Library/src/BNO055.d \
./Library/src/Controller.d \
./Library/src/RC.d \
./Library/src/Tamiya.d 


# Each subdirectory must supply rules for building sources it contributes
Library/src/%.o Library/src/%.su Library/src/%.cyclo: ../Library/src/%.c Library/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tanakon/Desktop/F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/Config/inc" -I"/home/tanakon/Desktop/F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/Library/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-src

clean-Library-2f-src:
	-$(RM) ./Library/src/AS5600.cyclo ./Library/src/AS5600.d ./Library/src/AS5600.o ./Library/src/AS5600.su ./Library/src/BNO055.cyclo ./Library/src/BNO055.d ./Library/src/BNO055.o ./Library/src/BNO055.su ./Library/src/Controller.cyclo ./Library/src/Controller.d ./Library/src/Controller.o ./Library/src/Controller.su ./Library/src/RC.cyclo ./Library/src/RC.d ./Library/src/RC.o ./Library/src/RC.su ./Library/src/Tamiya.cyclo ./Library/src/Tamiya.d ./Library/src/Tamiya.o ./Library/src/Tamiya.su

.PHONY: clean-Library-2f-src

