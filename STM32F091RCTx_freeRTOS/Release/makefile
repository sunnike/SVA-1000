################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
STM32F091RCTx_freeRTOS.hex \

SECONDARY_LIST += \
STM32F091RCTx_freeRTOS.lst \

SECONDARY_SIZE += \
STM32F091RCTx_freeRTOS.siz \


# All Target
all: STM32F091RCTx_freeRTOS.elf secondary-outputs

# Tool invocations
STM32F091RCTx_freeRTOS.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Werror -Wuninitialized -Wextra -Wmissing-declarations -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g -T "C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS\SW4STM32\STM32F091RCTx_freeRTOS\STM32F091RCTx_FLASH.ld" -Xlinker --gc-sections -Wl,-Map,"STM32F091RCTx_freeRTOS.map" --specs=nano.specs --specs=nosys.specs -o "STM32F091RCTx_freeRTOS.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

STM32F091RCTx_freeRTOS.hex: STM32F091RCTx_freeRTOS.elf
	@echo 'Invoking: GNU ARM Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "STM32F091RCTx_freeRTOS.elf"  "STM32F091RCTx_freeRTOS.hex"
	@echo 'Finished building: $@'
	@echo ' '

STM32F091RCTx_freeRTOS.lst: STM32F091RCTx_freeRTOS.elf
	@echo 'Invoking: GNU ARM Cross Create Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "STM32F091RCTx_freeRTOS.elf" > "STM32F091RCTx_freeRTOS.lst"
	@echo 'Finished building: $@'
	@echo ' '

STM32F091RCTx_freeRTOS.siz: STM32F091RCTx_freeRTOS.elf
	@echo 'Invoking: GNU ARM Cross Print Size'
	arm-none-eabi-size --format=berkeley "STM32F091RCTx_freeRTOS.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) STM32F091RCTx_freeRTOS.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
