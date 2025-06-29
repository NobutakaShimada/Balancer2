################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
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


BUILD_ARTIFACT_NAME := Balancer2_firm_Uart
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: Balancer2_firm_Uart.axf

# Tool invocations
Balancer2_firm_Uart.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/shimadalocal/src/Balancer2/MCUxpresso/Default_Balancer/Balancer2_firm_Uart/lib" -Xlinker --gc-sections -Xlinker -Map=Balancer2_firm_Uart.map -mcpu=cortex-m3 -mthumb -T "Balancer2_Debug.ld" -o "Balancer2_firm_Uart.axf" $(OBJS) $(USER_OBJS) $(LIBS) -lBalancer2_lib
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Balancer2_firm_Uart.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size Balancer2_firm_Uart.axf; arm-none-eabi-size Balancer2_firm_Uart.axf; arm-none-eabi-objcopy -O binary Balancer2_firm_Uart.axf Balancer2_firm_Uart.bin; checksum Balancer2_firm_Uart.bin;
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
