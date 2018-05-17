################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Werror -Wuninitialized -Wextra -Wmissing-declarations -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g -DUSE_HAL_DRIVER -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DSTM32F091xC -DARM_MATH_CM0 -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Inc" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Include" -I"C:/Users/Chenj/workspace/SVA-1000/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -std=gnu11 -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow -fno-sanitize-recover -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


