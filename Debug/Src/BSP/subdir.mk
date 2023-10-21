################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BSP/adc.c \
../Src/BSP/ak8963.c \
../Src/BSP/buzzer.c \
../Src/BSP/can.c \
../Src/BSP/delay.c \
../Src/BSP/flash.c \
../Src/BSP/i2c_device.c \
../Src/BSP/i2c_driver.c \
../Src/BSP/i2c_oled.c \
../Src/BSP/key.c \
../Src/BSP/led.c \
../Src/BSP/motor.c \
../Src/BSP/mpu9250.c \
../Src/BSP/nrf24l01.c \
../Src/BSP/power_control.c \
../Src/BSP/servo.c \
../Src/BSP/spi_flash.c \
../Src/BSP/uart4.c \
../Src/BSP/uart5.c \
../Src/BSP/usart1.c \
../Src/BSP/watchdog.c 

C_DEPS += \
./Src/BSP/adc.d \
./Src/BSP/ak8963.d \
./Src/BSP/buzzer.d \
./Src/BSP/can.d \
./Src/BSP/delay.d \
./Src/BSP/flash.d \
./Src/BSP/i2c_device.d \
./Src/BSP/i2c_driver.d \
./Src/BSP/i2c_oled.d \
./Src/BSP/key.d \
./Src/BSP/led.d \
./Src/BSP/motor.d \
./Src/BSP/mpu9250.d \
./Src/BSP/nrf24l01.d \
./Src/BSP/power_control.d \
./Src/BSP/servo.d \
./Src/BSP/spi_flash.d \
./Src/BSP/uart4.d \
./Src/BSP/uart5.d \
./Src/BSP/usart1.d \
./Src/BSP/watchdog.d 

OBJS += \
./Src/BSP/adc.o \
./Src/BSP/ak8963.o \
./Src/BSP/buzzer.o \
./Src/BSP/can.o \
./Src/BSP/delay.o \
./Src/BSP/flash.o \
./Src/BSP/i2c_device.o \
./Src/BSP/i2c_driver.o \
./Src/BSP/i2c_oled.o \
./Src/BSP/key.o \
./Src/BSP/led.o \
./Src/BSP/motor.o \
./Src/BSP/mpu9250.o \
./Src/BSP/nrf24l01.o \
./Src/BSP/power_control.o \
./Src/BSP/servo.o \
./Src/BSP/spi_flash.o \
./Src/BSP/uart4.o \
./Src/BSP/uart5.o \
./Src/BSP/usart1.o \
./Src/BSP/watchdog.o 


# Each subdirectory must supply rules for building sources it contributes
Src/BSP/%.o Src/BSP/%.su Src/BSP/%.cyclo: ../Src/BSP/%.c Src/BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -c -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/BSP/CMSIS/CM3/CoreSupport" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/EKF" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FatFs" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/FreeRTOS/include" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/System" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/STM32F10x_StdPeriph_Driver/inc" -I"D:/ProgramData/STM32CubeIDE/workspace_1.10.1/Tank_Dual_Pro/Src/User" -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-BSP

clean-Src-2f-BSP:
	-$(RM) ./Src/BSP/adc.cyclo ./Src/BSP/adc.d ./Src/BSP/adc.o ./Src/BSP/adc.su ./Src/BSP/ak8963.cyclo ./Src/BSP/ak8963.d ./Src/BSP/ak8963.o ./Src/BSP/ak8963.su ./Src/BSP/buzzer.cyclo ./Src/BSP/buzzer.d ./Src/BSP/buzzer.o ./Src/BSP/buzzer.su ./Src/BSP/can.cyclo ./Src/BSP/can.d ./Src/BSP/can.o ./Src/BSP/can.su ./Src/BSP/delay.cyclo ./Src/BSP/delay.d ./Src/BSP/delay.o ./Src/BSP/delay.su ./Src/BSP/flash.cyclo ./Src/BSP/flash.d ./Src/BSP/flash.o ./Src/BSP/flash.su ./Src/BSP/i2c_device.cyclo ./Src/BSP/i2c_device.d ./Src/BSP/i2c_device.o ./Src/BSP/i2c_device.su ./Src/BSP/i2c_driver.cyclo ./Src/BSP/i2c_driver.d ./Src/BSP/i2c_driver.o ./Src/BSP/i2c_driver.su ./Src/BSP/i2c_oled.cyclo ./Src/BSP/i2c_oled.d ./Src/BSP/i2c_oled.o ./Src/BSP/i2c_oled.su ./Src/BSP/key.cyclo ./Src/BSP/key.d ./Src/BSP/key.o ./Src/BSP/key.su ./Src/BSP/led.cyclo ./Src/BSP/led.d ./Src/BSP/led.o ./Src/BSP/led.su ./Src/BSP/motor.cyclo ./Src/BSP/motor.d ./Src/BSP/motor.o ./Src/BSP/motor.su ./Src/BSP/mpu9250.cyclo ./Src/BSP/mpu9250.d ./Src/BSP/mpu9250.o ./Src/BSP/mpu9250.su ./Src/BSP/nrf24l01.cyclo ./Src/BSP/nrf24l01.d ./Src/BSP/nrf24l01.o ./Src/BSP/nrf24l01.su ./Src/BSP/power_control.cyclo ./Src/BSP/power_control.d ./Src/BSP/power_control.o ./Src/BSP/power_control.su ./Src/BSP/servo.cyclo ./Src/BSP/servo.d ./Src/BSP/servo.o ./Src/BSP/servo.su ./Src/BSP/spi_flash.cyclo ./Src/BSP/spi_flash.d ./Src/BSP/spi_flash.o ./Src/BSP/spi_flash.su ./Src/BSP/uart4.cyclo ./Src/BSP/uart4.d ./Src/BSP/uart4.o ./Src/BSP/uart4.su ./Src/BSP/uart5.cyclo ./Src/BSP/uart5.d ./Src/BSP/uart5.o ./Src/BSP/uart5.su ./Src/BSP/usart1.cyclo ./Src/BSP/usart1.d ./Src/BSP/usart1.o ./Src/BSP/usart1.su ./Src/BSP/watchdog.cyclo ./Src/BSP/watchdog.d ./Src/BSP/watchdog.o ./Src/BSP/watchdog.su

.PHONY: clean-Src-2f-BSP

