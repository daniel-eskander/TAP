################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.c 

OBJS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.o 

C_DEPS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/%.o Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/%.su Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/%.cyclo: ../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/%.c Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-controller_tests

clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-controller_tests:
	-$(RM) ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_common_data.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/controller_test_group.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_reset_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/pid_tests.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/Common/src/controller_tests/sin_cos_tests.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-Common-2f-src-2f-controller_tests

