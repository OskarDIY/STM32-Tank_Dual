################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/EKF/double_matrix.c \
../Src/EKF/ekf.c \
../Src/EKF/ekf2.c \
../Src/EKF/fast_math.c \
../Src/EKF/matrix.c \
../Src/EKF/mini_matrix.c \
../Src/EKF/quaternion.c 

C_DEPS += \
./Src/EKF/double_matrix.d \
./Src/EKF/ekf.d \
./Src/EKF/ekf2.d \
./Src/EKF/fast_math.d \
./Src/EKF/matrix.d \
./Src/EKF/mini_matrix.d \
./Src/EKF/quaternion.d 

OBJS += \
./Src/EKF/double_matrix.o \
./Src/EKF/ekf.o \
./Src/EKF/ekf2.o \
./Src/EKF/fast_math.o \
./Src/EKF/matrix.o \
./Src/EKF/mini_matrix.o \
./Src/EKF/quaternion.o 


# Each subdirectory must supply rules for building sources it contributes
Src/EKF/%.o Src/EKF/%.su Src/EKF/%.cyclo: ../Src/EKF/%.c Src/EKF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-EKF

clean-Src-2f-EKF:
	-$(RM) ./Src/EKF/double_matrix.cyclo ./Src/EKF/double_matrix.d ./Src/EKF/double_matrix.o ./Src/EKF/double_matrix.su ./Src/EKF/ekf.cyclo ./Src/EKF/ekf.d ./Src/EKF/ekf.o ./Src/EKF/ekf.su ./Src/EKF/ekf2.cyclo ./Src/EKF/ekf2.d ./Src/EKF/ekf2.o ./Src/EKF/ekf2.su ./Src/EKF/fast_math.cyclo ./Src/EKF/fast_math.d ./Src/EKF/fast_math.o ./Src/EKF/fast_math.su ./Src/EKF/matrix.cyclo ./Src/EKF/matrix.d ./Src/EKF/matrix.o ./Src/EKF/matrix.su ./Src/EKF/mini_matrix.cyclo ./Src/EKF/mini_matrix.d ./Src/EKF/mini_matrix.o ./Src/EKF/mini_matrix.su ./Src/EKF/quaternion.cyclo ./Src/EKF/quaternion.d ./Src/EKF/quaternion.o ./Src/EKF/quaternion.su

.PHONY: clean-Src-2f-EKF

