################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f1xx/stm32f1xx_hal.c \
../system/src/stm32f1xx/stm32f1xx_hal_adc.c \
../system/src/stm32f1xx/stm32f1xx_hal_adc_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_cortex.c \
../system/src/stm32f1xx/stm32f1xx_hal_crc.c \
../system/src/stm32f1xx/stm32f1xx_hal_dma.c \
../system/src/stm32f1xx/stm32f1xx_hal_flash.c \
../system/src/stm32f1xx/stm32f1xx_hal_flash_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_gpio.c \
../system/src/stm32f1xx/stm32f1xx_hal_gpio_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_i2c.c \
../system/src/stm32f1xx/stm32f1xx_hal_pcd.c \
../system/src/stm32f1xx/stm32f1xx_hal_pcd_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_pwr.c \
../system/src/stm32f1xx/stm32f1xx_hal_rcc.c \
../system/src/stm32f1xx/stm32f1xx_hal_rcc_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_tim.c \
../system/src/stm32f1xx/stm32f1xx_hal_tim_ex.c \
../system/src/stm32f1xx/stm32f1xx_hal_uart.c \
../system/src/stm32f1xx/stm32f1xx_ll_usb.c 

OBJS += \
./system/src/stm32f1xx/stm32f1xx_hal.o \
./system/src/stm32f1xx/stm32f1xx_hal_adc.o \
./system/src/stm32f1xx/stm32f1xx_hal_adc_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_cortex.o \
./system/src/stm32f1xx/stm32f1xx_hal_crc.o \
./system/src/stm32f1xx/stm32f1xx_hal_dma.o \
./system/src/stm32f1xx/stm32f1xx_hal_flash.o \
./system/src/stm32f1xx/stm32f1xx_hal_flash_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_gpio.o \
./system/src/stm32f1xx/stm32f1xx_hal_gpio_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_i2c.o \
./system/src/stm32f1xx/stm32f1xx_hal_pcd.o \
./system/src/stm32f1xx/stm32f1xx_hal_pcd_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_pwr.o \
./system/src/stm32f1xx/stm32f1xx_hal_rcc.o \
./system/src/stm32f1xx/stm32f1xx_hal_rcc_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_tim.o \
./system/src/stm32f1xx/stm32f1xx_hal_tim_ex.o \
./system/src/stm32f1xx/stm32f1xx_hal_uart.o \
./system/src/stm32f1xx/stm32f1xx_ll_usb.o 

C_DEPS += \
./system/src/stm32f1xx/stm32f1xx_hal.d \
./system/src/stm32f1xx/stm32f1xx_hal_adc.d \
./system/src/stm32f1xx/stm32f1xx_hal_adc_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_cortex.d \
./system/src/stm32f1xx/stm32f1xx_hal_crc.d \
./system/src/stm32f1xx/stm32f1xx_hal_dma.d \
./system/src/stm32f1xx/stm32f1xx_hal_flash.d \
./system/src/stm32f1xx/stm32f1xx_hal_flash_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_gpio.d \
./system/src/stm32f1xx/stm32f1xx_hal_gpio_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_i2c.d \
./system/src/stm32f1xx/stm32f1xx_hal_pcd.d \
./system/src/stm32f1xx/stm32f1xx_hal_pcd_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_pwr.d \
./system/src/stm32f1xx/stm32f1xx_hal_rcc.d \
./system/src/stm32f1xx/stm32f1xx_hal_rcc_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_tim.d \
./system/src/stm32f1xx/stm32f1xx_hal_tim_ex.d \
./system/src/stm32f1xx/stm32f1xx_hal_uart.d \
./system/src/stm32f1xx/stm32f1xx_ll_usb.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f1xx/%.o: ../system/src/stm32f1xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1xx" -I"../system/include/cmsis/device" -I"../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"../Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc" -I"../Middlewares/Third_Party/FreeRTOS/Source/include" -I"../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\5.4 2016q3\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


