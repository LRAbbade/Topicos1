################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Example11.axf

# Tool invocations
Example11.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/CMSISv2p00_LPC17xx/Debug" -L"/Users/lucasabbade/Desktop/Inatel/Tópicos 1/FreeRTOS-exemplos-CMSISv2p00/FreeRTOS_Library/Debug" -L"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/Lib_EaBaseBoard/Debug" -L"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/Lib_MCU/Debug" -Xlinker --gc-sections -Xlinker -Map=Example11.map -mcpu=cortex-m3 -mthumb -T "Example11_Debug.ld" -o "Example11.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Example11.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size Example11.axf; # arm-none-eabi-objcopy -O ihex Example11.axf Example11.hex ;
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
