################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FatFs/diskio.c \
../Src/FatFs/ff.c 

C_DEPS += \
./Src/FatFs/diskio.d \
./Src/FatFs/ff.d 

OBJS += \
./Src/FatFs/diskio.o \
./Src/FatFs/ff.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FatFs/%.o Src/FatFs/%.su Src/FatFs/%.cyclo: ../Src/FatFs/%.c Src/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FatFs

clean-Src-2f-FatFs:
	-$(RM) ./Src/FatFs/diskio.cyclo ./Src/FatFs/diskio.d ./Src/FatFs/diskio.o ./Src/FatFs/diskio.su ./Src/FatFs/ff.cyclo ./Src/FatFs/ff.d ./Src/FatFs/ff.o ./Src/FatFs/ff.su

.PHONY: clean-Src-2f-FatFs

