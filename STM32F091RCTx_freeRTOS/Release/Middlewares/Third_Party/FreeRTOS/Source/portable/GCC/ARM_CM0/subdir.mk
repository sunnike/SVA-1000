################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Werror -Wuninitialized -Wextra -Wmissing-declarations -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g -DUSE_HAL_DRIVER -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DSTM32F091xC -DARM_MATH_CM0 -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Inc" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Drivers/CMSIS/Include" -I"D:/Git_workspace/SVA-1000/first_clone/STM32F091RCTx_freeRTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -std=gnu11 -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow -fno-sanitize-recover -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


