################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/src/Controller.c \
../Library/src/DC_Motor.c \
../Library/src/PAA5160E1.c 

OBJS += \
./Library/src/Controller.o \
./Library/src/DC_Motor.o \
./Library/src/PAA5160E1.o 

C_DEPS += \
./Library/src/Controller.d \
./Library/src/DC_Motor.d \
./Library/src/PAA5160E1.d 


# Each subdirectory must supply rules for building sources it contributes
Library/src/%.o Library/src/%.su Library/src/%.cyclo: ../Library/src/%.c Library/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tucha/OneDrive/Projects/Mobile Robot/F1TENTH_PROJECT/BNO_Calibrate/Library/inc" -I"C:/Users/tucha/OneDrive/Projects/Mobile Robot/F1TENTH_PROJECT/BNO_Calibrate/Config/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-src

clean-Library-2f-src:
	-$(RM) ./Library/src/Controller.cyclo ./Library/src/Controller.d ./Library/src/Controller.o ./Library/src/Controller.su ./Library/src/DC_Motor.cyclo ./Library/src/DC_Motor.d ./Library/src/DC_Motor.o ./Library/src/DC_Motor.su ./Library/src/PAA5160E1.cyclo ./Library/src/PAA5160E1.d ./Library/src/PAA5160E1.o ./Library/src/PAA5160E1.su

.PHONY: clean-Library-2f-src

