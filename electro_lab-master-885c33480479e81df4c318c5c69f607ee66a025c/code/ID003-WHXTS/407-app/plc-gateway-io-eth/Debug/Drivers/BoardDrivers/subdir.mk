################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BoardDrivers/DigitIO.c \
../Drivers/BoardDrivers/MCP3208.c \
../Drivers/BoardDrivers/flash.c \
../Drivers/BoardDrivers/oled.c \
../Drivers/BoardDrivers/sd2078.c 

OBJS += \
./Drivers/BoardDrivers/DigitIO.o \
./Drivers/BoardDrivers/MCP3208.o \
./Drivers/BoardDrivers/flash.o \
./Drivers/BoardDrivers/oled.o \
./Drivers/BoardDrivers/sd2078.o 

C_DEPS += \
./Drivers/BoardDrivers/DigitIO.d \
./Drivers/BoardDrivers/MCP3208.d \
./Drivers/BoardDrivers/flash.d \
./Drivers/BoardDrivers/oled.d \
./Drivers/BoardDrivers/sd2078.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BoardDrivers/%.o: ../Drivers/BoardDrivers/%.c Drivers/BoardDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"../Drivers/BoardDrivers" -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"D:/AX/electro_lab/code/407-app-eth/plc-gateway-io-eth/App/inc" -I"D:/AX/electro_lab/code/407-app-eth/plc-gateway-io-eth/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BoardDrivers

clean-Drivers-2f-BoardDrivers:
	-$(RM) ./Drivers/BoardDrivers/DigitIO.d ./Drivers/BoardDrivers/DigitIO.o ./Drivers/BoardDrivers/MCP3208.d ./Drivers/BoardDrivers/MCP3208.o ./Drivers/BoardDrivers/flash.d ./Drivers/BoardDrivers/flash.o ./Drivers/BoardDrivers/oled.d ./Drivers/BoardDrivers/oled.o ./Drivers/BoardDrivers/sd2078.d ./Drivers/BoardDrivers/sd2078.o

.PHONY: clean-Drivers-2f-BoardDrivers

