################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sent/bridge.c \
../Core/Src/sent/hal_stm32f042.c \
../Core/Src/sent/mode_manager.c \
../Core/Src/sent/sent_crc.c \
../Core/Src/sent/sent_decoder.c \
../Core/Src/sent/sent_encoder.c \
../Core/Src/sent/sent_protocol.c \
../Core/Src/sent/slcan.c 

OBJS += \
./Core/Src/sent/bridge.o \
./Core/Src/sent/hal_stm32f042.o \
./Core/Src/sent/mode_manager.o \
./Core/Src/sent/sent_crc.o \
./Core/Src/sent/sent_decoder.o \
./Core/Src/sent/sent_encoder.o \
./Core/Src/sent/sent_protocol.o \
./Core/Src/sent/slcan.o 

C_DEPS += \
./Core/Src/sent/bridge.d \
./Core/Src/sent/hal_stm32f042.d \
./Core/Src/sent/mode_manager.d \
./Core/Src/sent/sent_crc.d \
./Core/Src/sent/sent_decoder.d \
./Core/Src/sent/sent_encoder.d \
./Core/Src/sent/sent_protocol.d \
./Core/Src/sent/slcan.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sent/%.o Core/Src/sent/%.su Core/Src/sent/%.cyclo: ../Core/Src/sent/%.c Core/Src/sent/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sent

clean-Core-2f-Src-2f-sent:
	-$(RM) ./Core/Src/sent/bridge.cyclo ./Core/Src/sent/bridge.d ./Core/Src/sent/bridge.o ./Core/Src/sent/bridge.su ./Core/Src/sent/hal_stm32f042.cyclo ./Core/Src/sent/hal_stm32f042.d ./Core/Src/sent/hal_stm32f042.o ./Core/Src/sent/hal_stm32f042.su ./Core/Src/sent/mode_manager.cyclo ./Core/Src/sent/mode_manager.d ./Core/Src/sent/mode_manager.o ./Core/Src/sent/mode_manager.su ./Core/Src/sent/sent_crc.cyclo ./Core/Src/sent/sent_crc.d ./Core/Src/sent/sent_crc.o ./Core/Src/sent/sent_crc.su ./Core/Src/sent/sent_decoder.cyclo ./Core/Src/sent/sent_decoder.d ./Core/Src/sent/sent_decoder.o ./Core/Src/sent/sent_decoder.su ./Core/Src/sent/sent_encoder.cyclo ./Core/Src/sent/sent_encoder.d ./Core/Src/sent/sent_encoder.o ./Core/Src/sent/sent_encoder.su ./Core/Src/sent/sent_protocol.cyclo ./Core/Src/sent/sent_protocol.d ./Core/Src/sent/sent_protocol.o ./Core/Src/sent/sent_protocol.su ./Core/Src/sent/slcan.cyclo ./Core/Src/sent/slcan.d ./Core/Src/sent/slcan.o ./Core/Src/sent/slcan.su

.PHONY: clean-Core-2f-Src-2f-sent

