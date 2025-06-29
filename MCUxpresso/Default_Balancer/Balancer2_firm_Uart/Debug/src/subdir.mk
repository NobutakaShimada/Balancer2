################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/control.c \
../src/cr_startup_lpc13.c \
../src/main.c 

C_DEPS += \
./src/control.d \
./src/cr_startup_lpc13.d \
./src/main.d 

OBJS += \
./src/control.o \
./src/cr_startup_lpc13.o \
./src/main.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__USE_CMSIS=Balancer2_lib -DDEBUG -D__CODE_RED -D__REDLIB__ -I"/home/okui/mcuxpresso/BeuatoBalancer2/Balancer2/MCUxpresso/Balancer2_firm_Uart/lib" -I"/home/okui/mcuxpresso/BeuatoBalancer2/Balancer2/MCUxpresso/Balancer2_firm_Uart/inc" -O1 -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/control.d ./src/control.o ./src/cr_startup_lpc13.d ./src/cr_startup_lpc13.o ./src/main.d ./src/main.o

.PHONY: clean-src

