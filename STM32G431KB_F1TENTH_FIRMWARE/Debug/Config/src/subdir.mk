################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Config/src/Cytron_Motor_260rpm_250W.c 

OBJS += \
./Config/src/Cytron_Motor_260rpm_250W.o 

C_DEPS += \
./Config/src/Cytron_Motor_260rpm_250W.d 


# Each subdirectory must supply rules for building sources it contributes
Config/src/%.o Config/src/%.su Config/src/%.cyclo: ../Config/src/%.c Config/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/08809/Desktop/FIBO Online/FRA532 (Mobile Robot)/F1TENTH_PROJECT/STM32G431KB_F1TENTH_FIRMWARE/Library/inc" -I"C:/Users/08809/Desktop/FIBO Online/FRA532 (Mobile Robot)/F1TENTH_PROJECT/STM32G431KB_F1TENTH_FIRMWARE/Config/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Config-2f-src

clean-Config-2f-src:
	-$(RM) ./Config/src/Cytron_Motor_260rpm_250W.cyclo ./Config/src/Cytron_Motor_260rpm_250W.d ./Config/src/Cytron_Motor_260rpm_250W.o ./Config/src/Cytron_Motor_260rpm_250W.su

.PHONY: clean-Config-2f-src

