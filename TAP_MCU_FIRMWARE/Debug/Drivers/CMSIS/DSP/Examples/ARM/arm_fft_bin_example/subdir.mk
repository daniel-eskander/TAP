################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.c \
../Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.o \
./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.d \
./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/%.su Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/%.cyclo: ../Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_fft_bin_example

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_fft_bin_example:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.su ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_fft_bin_example

