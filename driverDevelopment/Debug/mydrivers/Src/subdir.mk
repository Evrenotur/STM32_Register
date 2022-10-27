################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mydrivers/Src/GPIO.c \
../mydrivers/Src/RCC.c 

OBJS += \
./mydrivers/Src/GPIO.o \
./mydrivers/Src/RCC.o 

C_DEPS += \
./mydrivers/Src/GPIO.d \
./mydrivers/Src/RCC.d 


# Each subdirectory must supply rules for building sources it contributes
mydrivers/Src/%.o mydrivers/Src/%.su: ../mydrivers/Src/%.c mydrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/STM32_ARGE/STM32Driver/driverDevelopment/mydrivers/Inc" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-mydrivers-2f-Src

clean-mydrivers-2f-Src:
	-$(RM) ./mydrivers/Src/GPIO.d ./mydrivers/Src/GPIO.o ./mydrivers/Src/GPIO.su ./mydrivers/Src/RCC.d ./mydrivers/Src/RCC.o ./mydrivers/Src/RCC.su

.PHONY: clean-mydrivers-2f-Src

