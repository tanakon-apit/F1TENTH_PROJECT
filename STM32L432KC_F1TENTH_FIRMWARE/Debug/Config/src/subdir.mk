################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Config/src/Mobile_Config.c 

OBJS += \
./Config/src/Mobile_Config.o 

C_DEPS += \
./Config/src/Mobile_Config.d 


# Each subdirectory must supply rules for building sources it contributes
Config/src/%.o Config/src/%.su Config/src/%.cyclo: ../Config/src/%.c Config/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tanakon/Desktop/F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/Config/inc" -I"/home/tanakon/Desktop/F1TENTH_PROJECT/STM32L432KC_F1TENTH_FIRMWARE/Library/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Config-2f-src

clean-Config-2f-src:
	-$(RM) ./Config/src/Mobile_Config.cyclo ./Config/src/Mobile_Config.d ./Config/src/Mobile_Config.o ./Config/src/Mobile_Config.su

.PHONY: clean-Config-2f-src

