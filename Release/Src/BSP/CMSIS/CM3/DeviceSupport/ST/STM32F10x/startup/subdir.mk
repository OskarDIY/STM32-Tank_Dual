################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.s 

S_DEPS += \
./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.d 

OBJS += \
./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.o 


# Each subdirectory must supply rules for building sources it contributes
Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/%.o: ../Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/%.s Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x-2f-startup

clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x-2f-startup:
	-$(RM) ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.d ./Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.o

.PHONY: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-DeviceSupport-2f-ST-2f-STM32F10x-2f-startup

