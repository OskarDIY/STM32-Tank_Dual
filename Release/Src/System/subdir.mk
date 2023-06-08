################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/System/filter.c \
../Src/System/obstacle.c \
../Src/System/radio.c \
../Src/System/sensors.c \
../Src/System/utils.c 

C_DEPS += \
./Src/System/filter.d \
./Src/System/obstacle.d \
./Src/System/radio.d \
./Src/System/sensors.d \
./Src/System/utils.d 

OBJS += \
./Src/System/filter.o \
./Src/System/obstacle.o \
./Src/System/radio.o \
./Src/System/sensors.o \
./Src/System/utils.o 


# Each subdirectory must supply rules for building sources it contributes
Src/System/%.o Src/System/%.su Src/System/%.cyclo: ../Src/System/%.c Src/System/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-System

clean-Src-2f-System:
	-$(RM) ./Src/System/filter.cyclo ./Src/System/filter.d ./Src/System/filter.o ./Src/System/filter.su ./Src/System/obstacle.cyclo ./Src/System/obstacle.d ./Src/System/obstacle.o ./Src/System/obstacle.su ./Src/System/radio.cyclo ./Src/System/radio.d ./Src/System/radio.o ./Src/System/radio.su ./Src/System/sensors.cyclo ./Src/System/sensors.d ./Src/System/sensors.o ./Src/System/sensors.su ./Src/System/utils.cyclo ./Src/System/utils.d ./Src/System/utils.o ./Src/System/utils.su

.PHONY: clean-Src-2f-System

