################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/display/Fonts/subdir.mk
-include Drivers/display/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BMP280/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
QueueMail_With_UART.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
QueueMail_With_UART.list \

OBJCOPY_BIN += \
QueueMail_With_UART.bin \


# All Target
all: QueueMail_With_UART.elf secondary-outputs

# Tool invocations
QueueMail_With_UART.elf: $(OBJS) $(USER_OBJS) C:\Users\MECHANIC\Desktop\RTOS\QueueMail_With_UART\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "QueueMail_With_UART.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\MECHANIC\Desktop\RTOS\QueueMail_With_UART\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="QueueMail_With_UART.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

QueueMail_With_UART.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "QueueMail_With_UART.list"
	@echo 'Finished building: $@'
	@echo ' '

QueueMail_With_UART.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "QueueMail_With_UART.bin"
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
