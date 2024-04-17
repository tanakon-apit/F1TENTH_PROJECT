################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/src/Controller.c \
../Library/src/DC_Motor.c \
../Library/src/Servo.c \
../Library/src/User_function.c \
../Library/src/pwm_freq.c 

OBJS += \
./Library/src/Controller.o \
./Library/src/DC_Motor.o \
./Library/src/Servo.o \
./Library/src/User_function.o \
./Library/src/pwm_freq.o 

C_DEPS += \
./Library/src/Controller.d \
./Library/src/DC_Motor.d \
./Library/src/Servo.d \
./Library/src/User_function.d \
./Library/src/pwm_freq.d 


# Each subdirectory must supply rules for building sources it contributes
Library/src/%.o Library/src/%.su Library/src/%.cyclo: ../Library/src/%.c Library/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kwan2/Documents/3-2 FIBO/FRA532 Mobile Robot/F1TENTH_PROJECT/STM32G431KB_F1TENTH_FIRMWARE/Library/inc" -I"C:/Users/kwan2/Documents/3-2 FIBO/FRA532 Mobile Robot/F1TENTH_PROJECT/STM32G431KB_F1TENTH_FIRMWARE/Config/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-src

clean-Library-2f-src:
	-$(RM) ./Library/src/Controller.cyclo ./Library/src/Controller.d ./Library/src/Controller.o ./Library/src/Controller.su ./Library/src/DC_Motor.cyclo ./Library/src/DC_Motor.d ./Library/src/DC_Motor.o ./Library/src/DC_Motor.su ./Library/src/Servo.cyclo ./Library/src/Servo.d ./Library/src/Servo.o ./Library/src/Servo.su ./Library/src/User_function.cyclo ./Library/src/User_function.d ./Library/src/User_function.o ./Library/src/User_function.su ./Library/src/pwm_freq.cyclo ./Library/src/pwm_freq.d ./Library/src/pwm_freq.o ./Library/src/pwm_freq.su

.PHONY: clean-Library-2f-src

