################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.c \
../Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.c 

C_DEPS += \
./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.d \
./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.d 

OBJS += \
./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.o \
./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/%.o Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/%.su: ../Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/%.c Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM33-2f-non_secure

clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM33-2f-non_secure:
	-$(RM) ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.d ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.o ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/port.su ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.d ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.o ./Src/FreeRTOS/portable/GCC/ARM_CM33/non_secure/portasm.su

.PHONY: clean-Src-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM33-2f-non_secure

