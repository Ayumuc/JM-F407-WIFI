################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/src/MachineStatusManager.c \
../App/src/boardConfig.c \
../App/src/dataCollector.c \
../App/src/display.c \
../App/src/logQueue.c \
../App/src/modbusTcp.c \
../App/src/rs485.c \
../App/src/tcpServer.c 

OBJS += \
./App/src/MachineStatusManager.o \
./App/src/boardConfig.o \
./App/src/dataCollector.o \
./App/src/display.o \
./App/src/logQueue.o \
./App/src/modbusTcp.o \
./App/src/rs485.o \
./App/src/tcpServer.o 

C_DEPS += \
./App/src/MachineStatusManager.d \
./App/src/boardConfig.d \
./App/src/dataCollector.d \
./App/src/display.d \
./App/src/logQueue.d \
./App/src/modbusTcp.d \
./App/src/rs485.d \
./App/src/tcpServer.d 


# Each subdirectory must supply rules for building sources it contributes
App/src/%.o: ../App/src/%.c App/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"../Drivers/BoardDrivers" -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"F:/stm32cube/electro_lab-master-885c33480479e81df4c318c5c69f607ee66a025c/electro_lab-master-885c33480479e81df4c318c5c69f607ee66a025c/code/ID003-WHXTS/407-app/plc-gateway-io-eth-wifi/App/inc" -I"F:/stm32cube/electro_lab-master-885c33480479e81df4c318c5c69f607ee66a025c/electro_lab-master-885c33480479e81df4c318c5c69f607ee66a025c/code/ID003-WHXTS/407-app/plc-gateway-io-eth-wifi/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-src

clean-App-2f-src:
	-$(RM) ./App/src/MachineStatusManager.d ./App/src/MachineStatusManager.o ./App/src/boardConfig.d ./App/src/boardConfig.o ./App/src/dataCollector.d ./App/src/dataCollector.o ./App/src/display.d ./App/src/display.o ./App/src/logQueue.d ./App/src/logQueue.o ./App/src/modbusTcp.d ./App/src/modbusTcp.o ./App/src/rs485.d ./App/src/rs485.o ./App/src/tcpServer.d ./App/src/tcpServer.o

.PHONY: clean-App-2f-src

