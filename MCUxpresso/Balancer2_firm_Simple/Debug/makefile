################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Balancer2_firm_Simple
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: Balancer2_firm_Simple.axf

# Tool invocations
Balancer2_firm_Simple.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"F:/BeuatoBalancer/Balancer2/MCUxpresso/Balancer2_firm_Simple/lib" -Xlinker --gc-sections -Xlinker -Map=Balancer2_firm_Simple.map -mcpu=cortex-m3 -mthumb -T "Balancer2_Debug.ld" -o "Balancer2_firm_Simple.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Balancer2_firm_Simple.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size Balancer2_firm_Simple.axf; arm-none-eabi-size Balancer2_firm_Simple.axf; arm-none-eabi-objcopy -O binary Balancer2_firm_Simple.axf Balancer2_firm_Simple.bin; checksum Balancer2_firm_Simple.bin;
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
