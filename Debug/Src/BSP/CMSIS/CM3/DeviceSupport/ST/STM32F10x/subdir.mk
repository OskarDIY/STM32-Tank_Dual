################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c 

C_DEPS += \
./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.d 

OBJS += \
./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.o 


# Each subdirectory must supply rules for building sources it contributes
Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.o Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.su Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.cyclo: ../Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.c Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x

clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x:
	-$(RM) ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.cyclo ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.d ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.o ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.su

.PHONY: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x

