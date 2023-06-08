################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FatFs-2f-option

clean-Src-2f-FatFs-2f-option:
	-$(RM) ./Src/FatFs/option/cc936.cyclo ./Src/FatFs/option/cc936.d ./Src/FatFs/option/cc936.o ./Src/FatFs/option/cc936.su ./Src/FatFs/option/syscall.cyclo ./Src/FatFs/option/syscall.d ./Src/FatFs/option/syscall.o ./Src/FatFs/option/syscall.su

.PHONY: clean-Src-2f-FatFs-2f-option

