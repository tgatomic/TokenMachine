################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/TWI_LCD.c \
../src/freertos.c \
../src/main.c \
../src/stm32f1xx_hal_msp.c \
../src/stm32f1xx_hal_timebase_TIM.c \
../src/stm32f1xx_it.c \
../src/usb_device.c \
../src/usbd_conf.c \
../src/usbd_desc.c \
../src/usbd_dfu_if.c 

OBJS += \
./src/TWI_LCD.o \
./src/freertos.o \
./src/main.o \
./src/stm32f1xx_hal_msp.o \
./src/stm32f1xx_hal_timebase_TIM.o \
./src/stm32f1xx_it.o \
./src/usb_device.o \
./src/usbd_conf.o \
./src/usbd_desc.o \
./src/usbd_dfu_if.o 

C_DEPS += \
./src/TWI_LCD.d \
./src/freertos.d \
./src/main.d \
./src/stm32f1xx_hal_msp.d \
./src/stm32f1xx_hal_timebase_TIM.d \
./src/stm32f1xx_it.d \
./src/usb_device.d \
./src/usbd_conf.d \
./src/usbd_desc.d \
./src/usbd_dfu_if.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1xx" -I"../system/include/cmsis/device" -I"../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"../Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc" -I"../Middlewares/Third_Party/FreeRTOS/Source/include" -I"../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\5.4 2016q3\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


