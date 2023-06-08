################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOS/croutine.c \
../Src/FreeRTOS/event_groups.c \
../Src/FreeRTOS/list.c \
../Src/FreeRTOS/queue.c \
../Src/FreeRTOS/tasks.c \
../Src/FreeRTOS/timers.c 

C_DEPS += \
./Src/FreeRTOS/croutine.d \
./Src/FreeRTOS/event_groups.d \
./Src/FreeRTOS/list.d \
./Src/FreeRTOS/queue.d \
./Src/FreeRTOS/tasks.d \
./Src/FreeRTOS/timers.d 

OBJS += \
./Src/FreeRTOS/croutine.o \
./Src/FreeRTOS/event_groups.o \
./Src/FreeRTOS/list.o \
./Src/FreeRTOS/queue.o \
./Src/FreeRTOS/tasks.o \
./Src/FreeRTOS/timers.o 


# Each subdirectory must supply rules for building sources it contributes
Src/FreeRTOS/%.o Src/FreeRTOS/%.su Src/FreeRTOS/%.cyclo: ../Src/FreeRTOS/%.c Src/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -c -I../Inc -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-FreeRTOS

clean-Src-2f-FreeRTOS:
	-$(RM) ./Src/FreeRTOS/croutine.cyclo ./Src/FreeRTOS/croutine.d ./Src/FreeRTOS/croutine.o ./Src/FreeRTOS/croutine.su ./Src/FreeRTOS/event_groups.cyclo ./Src/FreeRTOS/event_groups.d ./Src/FreeRTOS/event_groups.o ./Src/FreeRTOS/event_groups.su ./Src/FreeRTOS/list.cyclo ./Src/FreeRTOS/list.d ./Src/FreeRTOS/list.o ./Src/FreeRTOS/list.su ./Src/FreeRTOS/queue.cyclo ./Src/FreeRTOS/queue.d ./Src/FreeRTOS/queue.o ./Src/FreeRTOS/queue.su ./Src/FreeRTOS/tasks.cyclo ./Src/FreeRTOS/tasks.d ./Src/FreeRTOS/tasks.o ./Src/FreeRTOS/tasks.su ./Src/FreeRTOS/timers.cyclo ./Src/FreeRTOS/timers.d ./Src/FreeRTOS/timers.o ./Src/FreeRTOS/timers.su

.PHONY: clean-Src-2f-FreeRTOS

