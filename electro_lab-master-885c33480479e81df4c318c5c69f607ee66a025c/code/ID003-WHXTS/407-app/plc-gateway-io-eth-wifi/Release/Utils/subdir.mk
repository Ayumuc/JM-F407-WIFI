################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utils/cJSON.c \
../Utils/cpuId.c \
../Utils/cpu_utils.c \
../Utils/dataFilter.c \
../Utils/delay.c \
../Utils/iic.c \
../Utils/ipUtils.c \
../Utils/myfunction.c \
../Utils/ring_fifo.c 

OBJS += \
./Utils/cJSON.o \
./Utils/cpuId.o \
./Utils/cpu_utils.o \
./Utils/dataFilter.o \
./Utils/delay.o \
./Utils/iic.o \
./Utils/ipUtils.o \
./Utils/myfunction.o \
./Utils/ring_fifo.o 

C_DEPS += \
./Utils/cJSON.d \
./Utils/cpuId.d \
./Utils/cpu_utils.d \
./Utils/dataFilter.d \
./Utils/delay.d \
./Utils/iic.d \
./Utils/ipUtils.d \
./Utils/myfunction.d \
./Utils/ring_fifo.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/%.o: ../Utils/%.c Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utils

clean-Utils:
	-$(RM) ./Utils/cJSON.d ./Utils/cJSON.o ./Utils/cpuId.d ./Utils/cpuId.o ./Utils/cpu_utils.d ./Utils/cpu_utils.o ./Utils/dataFilter.d ./Utils/dataFilter.o ./Utils/delay.d ./Utils/delay.o ./Utils/iic.d ./Utils/iic.o ./Utils/ipUtils.d ./Utils/ipUtils.o ./Utils/myfunction.d ./Utils/myfunction.o ./Utils/ring_fifo.d ./Utils/ring_fifo.o

.PHONY: clean-Utils

