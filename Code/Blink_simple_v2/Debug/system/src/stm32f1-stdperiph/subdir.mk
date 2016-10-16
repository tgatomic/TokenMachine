################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f1-stdperiph/misc.c \
../system/src/stm32f1-stdperiph/stm32f10x_gpio.c \
../system/src/stm32f1-stdperiph/stm32f10x_rcc.c 

OBJS += \
./system/src/stm32f1-stdperiph/misc.o \
./system/src/stm32f1-stdperiph/stm32f10x_gpio.o \
./system/src/stm32f1-stdperiph/stm32f10x_rcc.o 

C_DEPS += \
./system/src/stm32f1-stdperiph/misc.d \
./system/src/stm32f1-stdperiph/stm32f10x_gpio.d \
./system/src/stm32f1-stdperiph/stm32f10x_rcc.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f1-stdperiph/%.o: ../system/src/stm32f1-stdperiph/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -I../system/include/stm32f1xx -I../system/include/cmsis/device -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"C:\Program Files (x86)\GNU Tools ARM Embedded\5.4 2016q3\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


