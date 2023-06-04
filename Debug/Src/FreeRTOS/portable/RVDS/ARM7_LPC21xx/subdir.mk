################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/portASM.s 

C_SRCS += \
../Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.c 

S_DEPS += \
./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/portASM.d 

C_DEPS += \
./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.d 

OBJS += \
./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.o \
./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/portASM.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/%.o Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/%.su: ../Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/%.c Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/User" -I"D:/ProgramData/Keil_ARM/Tank_Dual_Pro/Src/FreeRTOS/portable/RVDS/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/%.o: ../Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/%.s Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Src-2f-FreeRTOS-2f-portable-2f-RVDS-2f-ARM7_LPC21xx

clean-Src-2f-FreeRTOS-2f-portable-2f-RVDS-2f-ARM7_LPC21xx:
	-$(RM) ./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.d ./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.o ./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/port.su ./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/portASM.d ./Src/FreeRTOS/portable/RVDS/ARM7_LPC21xx/portASM.o

.PHONY: clean-Src-2f-FreeRTOS-2f-portable-2f-RVDS-2f-ARM7_LPC21xx

