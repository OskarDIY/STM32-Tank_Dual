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
Src/BSP/CMSIS/CM3/CoreSupport/%.o Src/BSP/CMSIS/CM3/CoreSupport/%.su Src/BSP/CMSIS/CM3/CoreSupport/%.cyclo: ../Src/BSP/CMSIS/CM3/CoreSupport/%.c Src/BSP/CMSIS/CM3/CoreSupport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport

clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport:
	-$(RM) ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.cyclo ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.d ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.o ./Src/BSP/CMSIS/CM3/CoreSupport/core_cm3.su

.PHONY: clean-Src-2f-BSP-2f-CMSIS-2f-CM3-2f-CoreSupport

