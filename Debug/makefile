################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
driver_stm32f407xx.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
driver_stm32f407xx.list \

OBJCOPY_BIN += \
driver_stm32f407xx.bin \


# All Target
all: driver_stm32f407xx.elf secondary-outputs

# Tool invocations
driver_stm32f407xx.elf: $(OBJS) $(USER_OBJS) /home/huongnguyen/Desktop/drivers_stm32f407xx/driver/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "driver_stm32f407xx.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/huongnguyen/Desktop/drivers_stm32f407xx/driver/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="driver_stm32f407xx.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

driver_stm32f407xx.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "driver_stm32f407xx.list"
	@echo 'Finished building: $@'
	@echo ' '

driver_stm32f407xx.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "driver_stm32f407xx.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
