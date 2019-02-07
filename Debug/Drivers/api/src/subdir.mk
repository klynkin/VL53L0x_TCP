################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/api/src/vl53l0x_api.c \
../Drivers/api/src/vl53l0x_api_calibration.c \
../Drivers/api/src/vl53l0x_api_core.c \
../Drivers/api/src/vl53l0x_api_ranging.c \
../Drivers/api/src/vl53l0x_api_strings.c \
../Drivers/api/src/vl53l0x_platform.c 

OBJS += \
./Drivers/api/src/vl53l0x_api.o \
./Drivers/api/src/vl53l0x_api_calibration.o \
./Drivers/api/src/vl53l0x_api_core.o \
./Drivers/api/src/vl53l0x_api_ranging.o \
./Drivers/api/src/vl53l0x_api_strings.o \
./Drivers/api/src/vl53l0x_platform.o 

C_DEPS += \
./Drivers/api/src/vl53l0x_api.d \
./Drivers/api/src/vl53l0x_api_calibration.d \
./Drivers/api/src/vl53l0x_api_core.d \
./Drivers/api/src/vl53l0x_api_ranging.d \
./Drivers/api/src/vl53l0x_api_strings.d \
./Drivers/api/src/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/api/src/vl53l0x_api.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_api.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/api/src/vl53l0x_api_calibration.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_api_calibration.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/api/src/vl53l0x_api_core.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_api_core.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/api/src/vl53l0x_api_ranging.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_api_ranging.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/api/src/vl53l0x_api_strings.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_api_strings.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/api/src/vl53l0x_platform.o: C:/Keil_v5/cube/ethernet/Drivers/api/src/vl53l0x_platform.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/ethernet/Drivers/api" -I"C:/Keil_v5/cube/ethernet/Drivers/api/inc" -I"C:/Keil_v5/cube/ethernet/Drivers/api/src" -I"C:/Keil_v5/cube/ethernet/Inc" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/ethernet/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/ethernet/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/ethernet/Drivers/CMSIS/Include"  -O1 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


