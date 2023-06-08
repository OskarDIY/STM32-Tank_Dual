################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/User/main.c \
../Src/User/stm32f10x_it.c 

C_DEPS += \
./Src/User/main.d \
./Src/User/stm32f10x_it.d 

OBJS += \
./Src/User/main.o \
./Src/User/stm32f10x_it.o 


# Each subdirectory must supply rules for building sources it contributes
Src/User/%.o Src/User/%.su Src/User/%.cyclo: ../Src/User/%.c Src/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-User

clean-Src-2f-User:
	-$(RM) ./Src/User/main.cyclo ./Src/User/main.d ./Src/User/main.o ./Src/User/main.su ./Src/User/stm32f10x_it.cyclo ./Src/User/stm32f10x_it.d ./Src/User/stm32f10x_it.o ./Src/User/stm32f10x_it.su

.PHONY: clean-Src-2f-User

