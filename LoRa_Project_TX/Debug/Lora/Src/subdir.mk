################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lora/Src/lr_fhss_mac.c \
../Lora/Src/sx1262_B_common.c \
../Lora/Src/sx126x.c \
../Lora/Src/sx126x_driver_version.c \
../Lora/Src/sx126x_hal.c \
../Lora/Src/sx126x_lr_fhss.c 

OBJS += \
./Lora/Src/lr_fhss_mac.o \
./Lora/Src/sx1262_B_common.o \
./Lora/Src/sx126x.o \
./Lora/Src/sx126x_driver_version.o \
./Lora/Src/sx126x_hal.o \
./Lora/Src/sx126x_lr_fhss.o 

C_DEPS += \
./Lora/Src/lr_fhss_mac.d \
./Lora/Src/sx1262_B_common.d \
./Lora/Src/sx126x.d \
./Lora/Src/sx126x_driver_version.d \
./Lora/Src/sx126x_hal.d \
./Lora/Src/sx126x_lr_fhss.d 


# Each subdirectory must supply rules for building sources it contributes
Lora/Src/%.o Lora/Src/%.su Lora/Src/%.cyclo: ../Lora/Src/%.c Lora/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G070xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/adamm/STM32CubeIDE/workspace_1.14.1/LoRa_Project_TX/Lora/Inc" -I"C:/Users/adamm/STM32CubeIDE/workspace_1.14.1/LoRa_Project_TX/Lora/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lora-2f-Src

clean-Lora-2f-Src:
	-$(RM) ./Lora/Src/lr_fhss_mac.cyclo ./Lora/Src/lr_fhss_mac.d ./Lora/Src/lr_fhss_mac.o ./Lora/Src/lr_fhss_mac.su ./Lora/Src/sx1262_B_common.cyclo ./Lora/Src/sx1262_B_common.d ./Lora/Src/sx1262_B_common.o ./Lora/Src/sx1262_B_common.su ./Lora/Src/sx126x.cyclo ./Lora/Src/sx126x.d ./Lora/Src/sx126x.o ./Lora/Src/sx126x.su ./Lora/Src/sx126x_driver_version.cyclo ./Lora/Src/sx126x_driver_version.d ./Lora/Src/sx126x_driver_version.o ./Lora/Src/sx126x_driver_version.su ./Lora/Src/sx126x_hal.cyclo ./Lora/Src/sx126x_hal.d ./Lora/Src/sx126x_hal.o ./Lora/Src/sx126x_hal.su ./Lora/Src/sx126x_lr_fhss.cyclo ./Lora/Src/sx126x_lr_fhss.d ./Lora/Src/sx126x_lr_fhss.o ./Lora/Src/sx126x_lr_fhss.su

.PHONY: clean-Lora-2f-Src

