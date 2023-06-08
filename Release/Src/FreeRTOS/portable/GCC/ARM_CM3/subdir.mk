################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOS/portable/GCC/ARM_CM3/port.c 

C_DEPS += \
./Src/FreeRTOS/portable/GCC/ARM_CM3/port.d 

OBJS += \
./Src/FreeRTOS/portable/GCC/ARM_CM3/port.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/portable/GCC/ARM_CM3/%.o Src/FreeRTOS/portable/GCC/ARM_CM3/%.su Src/FreeRTOS/portable/GCC/ARM_CM3/%.cyclo: ../Src/FreeRTOS/portable/GCC/ARM_CM3/%.c Src/FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3

clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3:
	-$(RM) ./Src/FreeRTOS/portable/GCC/ARM_CM3/port.cyclo ./Src/FreeRTOS/portable/GCC/ARM_CM3/port.d ./Src/FreeRTOS/portable/GCC/ARM_CM3/port.o ./Src/FreeRTOS/portable/GCC/ARM_CM3/port.su

.PHONY: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3

