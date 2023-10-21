################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FatFs/option/cc936.c \
../Src/FatFs/option/syscall.c 

C_DEPS += \
./Src/FatFs/option/cc936.d \
./Src/FatFs/option/syscall.d 

OBJS += \
./Src/FatFs/option/cc936.o \
./Src/FatFs/option/syscall.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FatFs/option/%.o Src/FatFs/option/%.su Src/FatFs/option/%.cyclo: ../Src/FatFs/option/%.c Src/FatFs/option/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FatFs-2f-option

clean-Src-2f-FatFs-2f-option:
	-$(RM) ./Src/FatFs/option/cc936.cyclo ./Src/FatFs/option/cc936.d ./Src/FatFs/option/cc936.o ./Src/FatFs/option/cc936.su ./Src/FatFs/option/syscall.cyclo ./Src/FatFs/option/syscall.d ./Src/FatFs/option/syscall.o ./Src/FatFs/option/syscall.su

.PHONY: clean-Src-2f-FatFs-2f-option

