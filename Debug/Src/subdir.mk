################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ethernetif.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/lwip.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/tim.c \
../Src/usart.c \
../Src/vl53l0x_calibration.c 

OBJS += \
./Src/ethernetif.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/lwip.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/tim.o \
./Src/usart.o \
./Src/vl53l0x_calibration.o 

C_DEPS += \
./Src/ethernetif.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/lwip.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/tim.d \
./Src/usart.d \
./Src/vl53l0x_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


