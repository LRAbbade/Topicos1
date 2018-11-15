################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cr_startup_lpc17.c \
../main.c 

OBJS += \
./cr_startup_lpc17.o \
./main.o 

C_DEPS += \
./cr_startup_lpc17.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv2p00_LPC17xx -D__CODE_RED -D__REDLIB__ -I"/Users/lucasabbade/Desktop/Inatel/Tópicos 1/FreeRTOS-exemplos-CMSISv2p00/FreeRTOS_Library/demo_code" -I"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/CMSISv2p00_LPC17xx/inc" -I"/Users/lucasabbade/Desktop/Inatel/Tópicos 1/FreeRTOS-exemplos-CMSISv2p00/FreeRTOS_Library/include" -I"/Users/lucasabbade/Desktop/Inatel/Tópicos 1/FreeRTOS-exemplos-CMSISv2p00/FreeRTOS_Library/portable" -I"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/Lib_EaBaseBoard/inc" -I"/Users/lucasabbade/Documents/LPCXpresso_8.2.2/workspace/Lib_MCU/inc" -O0 -g3 -fsigned-char -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


