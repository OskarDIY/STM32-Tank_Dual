################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.c 

C_DEPS += \
./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.d 

OBJS += \
./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.o 


# Each subdirectory must supply rules for building sources it contributes
Src/BSP/CMSIS/CM3/CoreSupport/%.o Src/BSP/CMSIS/CM3/CoreSupport/%.su: ../Src/BSP/CMSIS/CM3/CoreSupport/%.c Src/BSP/CMSIS/CM3/CoreSupport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport

clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport:
	-$(RM) ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.d ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.o ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.su

.PHONY: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport

