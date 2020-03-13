################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/mpu/eMPL/inv_mpu.c \
../Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Drivers/mpu/eMPL/inv_mpu.o \
./Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Drivers/mpu/eMPL/inv_mpu.d \
./Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/mpu/eMPL/inv_mpu.o: ../Drivers/mpu/eMPL/inv_mpu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/3DBox -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/display -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/mpu -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/mpu/eMPL/inv_mpu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.o: ../Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/3DBox -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/display -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/mpu -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/mpu/eMPL/inv_mpu_dmp_motion_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

