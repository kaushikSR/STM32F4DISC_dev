################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"/media/kaushik/localDiskB/MasteringMCU/systemWorkbench/workspace/clock_Switch/StdPeriph_Driver/inc" -I"/media/kaushik/localDiskB/MasteringMCU/systemWorkbench/workspace/clock_Switch/inc" -I"/media/kaushik/localDiskB/MasteringMCU/systemWorkbench/workspace/clock_Switch/CMSIS/device" -I"/media/kaushik/localDiskB/MasteringMCU/systemWorkbench/workspace/clock_Switch/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


