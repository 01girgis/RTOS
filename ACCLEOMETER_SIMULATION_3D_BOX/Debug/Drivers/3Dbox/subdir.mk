################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/3Dbox/3Dbox.c 

OBJS += \
./Drivers/3Dbox/3Dbox.o 

C_DEPS += \
./Drivers/3Dbox/3Dbox.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/3Dbox/3Dbox.o: ../Drivers/3Dbox/3Dbox.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/3DBox -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/display -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/mpu -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/3Dbox/3Dbox.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

