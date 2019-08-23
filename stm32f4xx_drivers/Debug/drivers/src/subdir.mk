################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407xx_gpio.c \
../drivers/src/stm32f407xx_i2c.c \
../drivers/src/stm32f407xx_rcc.c \
../drivers/src/stm32f407xx_spi.c 

OBJS += \
./drivers/src/stm32f407xx_gpio.o \
./drivers/src/stm32f407xx_i2c.o \
./drivers/src/stm32f407xx_rcc.o \
./drivers/src/stm32f407xx_spi.o 

C_DEPS += \
./drivers/src/stm32f407xx_gpio.d \
./drivers/src/stm32f407xx_i2c.d \
./drivers/src/stm32f407xx_rcc.d \
./drivers/src/stm32f407xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"/media/kaushik/localDiskB/MasteringMCU/systemWorkbench/workspace/stm32f4xx_drivers/drivers/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


