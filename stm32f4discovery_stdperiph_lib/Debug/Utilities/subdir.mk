################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/stm32f4_discovery.c \
../Utilities/stm32f4_discovery_lis302dl.c 

OBJS += \
./Utilities/stm32f4_discovery.o \
./Utilities/stm32f4_discovery_lis302dl.o 

C_DEPS += \
./Utilities/stm32f4_discovery.d \
./Utilities/stm32f4_discovery_lis302dl.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/%.o: ../Utilities/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4DISCOVERY -DSTM32F407VGTx -DSTM32 -DSTM32F4 -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -DSTM32F40_41xxx -I"/Users/marcospanghero/Documents/dev/workspace/stm32f4discovery_stdperiph_lib/CMSIS/core" -I"/Users/marcospanghero/Documents/dev/workspace/stm32f4discovery_stdperiph_lib/CMSIS/device" -I"/Users/marcospanghero/Documents/dev/workspace/stm32f4discovery_stdperiph_lib/StdPeriph_Driver/inc" -I"/Users/marcospanghero/Documents/dev/workspace/stm32f4discovery_stdperiph_lib/Utilities" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


