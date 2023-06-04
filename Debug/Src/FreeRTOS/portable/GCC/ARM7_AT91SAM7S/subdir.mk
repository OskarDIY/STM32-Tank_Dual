################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.c \
../Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.c \
../Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.c 

C_DEPS += \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.d \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.d \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.d 

OBJS += \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.o \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.o \
./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/%.o Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/%.su: ../Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/%.c Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM7_AT91SAM7S

clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM7_AT91SAM7S:
	-$(RM) ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.d ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.o ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.su ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.d ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.o ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/port.su ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.d ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.o ./Src/FreeRTOS/portable/GCC/ARM7_AT91SAM7S/portISR.su

.PHONY: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM7_AT91SAM7S

