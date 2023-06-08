################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOS/portable/MemMang/heap_4.c 

C_DEPS += \
./Src/FreeRTOS/portable/MemMang/heap_4.d 

OBJS += \
./Src/FreeRTOS/portable/MemMang/heap_4.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/portable/MemMang/%.o Src/FreeRTOS/portable/MemMang/%.su Src/FreeRTOS/portable/MemMang/%.cyclo: ../Src/FreeRTOS/portable/MemMang/%.c Src/FreeRTOS/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FreeRTOS-2f-portable-2f-MemMang

clean-Src-2f-FreeRTOS-2f-portable-2f-MemMang:
	-$(RM) ./Src/FreeRTOS/portable/MemMang/heap_4.cyclo ./Src/FreeRTOS/portable/MemMang/heap_4.d ./Src/FreeRTOS/portable/MemMang/heap_4.o ./Src/FreeRTOS/portable/MemMang/heap_4.su

.PHONY: clean-Src-2f-FreeRTOS-2f-portable-2f-MemMang

