################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/User/subdir.mk
-include Src/System/subdir.mk
-include Src/STM32F10x_StdPeriph_Driver/src/subdir.mk
-include Src/FreeRTOS/portable/MemMang/subdir.mk
-include Src/FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
-include Src/FreeRTOS/subdir.mk
-include Src/FatFs/option/subdir.mk
-include Src/FatFs/subdir.mk
-include Src/EKF/subdir.mk
-include Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/subdir.mk
-include Src/BSP/CMSIS/CM3/DeviceSupport/ST/STM32F10x/subdir.mk
-include Src/BSP/CMSIS/CM3/CoreSupport/subdir.mk
-include Src/BSP/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Tank_Dual_Pro
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Tank_Dual_Pro.elf \

MAP_FILES += \
Tank_Dual_Pro.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Tank_Dual_Pro.list \


# All Target
all: main-build

# Main-build Target
main-build: Tank_Dual_Pro.elf secondary-outputs

# Tool invocations
Tank_Dual_Pro.elf Tank_Dual_Pro.map: $(OBJS) $(USER_OBJS) D:\ProgramData\STM32CubeIDE\workspace_1.10.1\Tank_Dual_Pro\STM32F103RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-g++ -o "Tank_Dual_Pro.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\ProgramData\STM32CubeIDE\workspace_1.10.1\Tank_Dual_Pro\STM32F103RCTX_FLASH.ld" -Wl,-Map="Tank_Dual_Pro.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Tank_Dual_Pro.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Tank_Dual_Pro.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Tank_Dual_Pro.elf Tank_Dual_Pro.list Tank_Dual_Pro.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
