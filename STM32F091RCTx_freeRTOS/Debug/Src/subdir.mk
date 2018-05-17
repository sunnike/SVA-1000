################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../Src/startup_stm32f091xc.asm 

C_SRCS += \
../Src/adc.c \
../Src/aewin_def.c \
../Src/can.c \
../Src/dma.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/iwdg.c \
../Src/main.c \
../Src/rtc.c \
../Src/spi.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_hal_timebase_TIM.c \
../Src/stm32f0xx_it.c \
../Src/system_stm32f0xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/aewin_def.o \
./Src/can.o \
./Src/dma.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/iwdg.o \
./Src/main.o \
./Src/rtc.o \
./Src/spi.o \
./Src/startup_stm32f091xc.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_hal_timebase_TIM.o \
./Src/stm32f0xx_it.o \
./Src/system_stm32f0xx.o \
./Src/tim.o \
./Src/usart.o 

ASM_DEPS += \
./Src/startup_stm32f091xc.d 

C_DEPS += \
./Src/adc.d \
./Src/aewin_def.d \
./Src/can.d \
./Src/dma.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/iwdg.d \
./Src/main.d \
./Src/rtc.d \
./Src/spi.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_hal_timebase_TIM.d \
./Src/stm32f0xx_it.d \
./Src/system_stm32f0xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Werror -Wuninitialized -Wextra -Wmissing-declarations -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g3 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DSTM32F091xC -DARM_MATH_CM0 -DUSE_HAL_DRIVER -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Inc" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -std=gnu11 -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow -fno-sanitize-recover -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Werror -Wuninitialized -Wextra -Wmissing-declarations -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


