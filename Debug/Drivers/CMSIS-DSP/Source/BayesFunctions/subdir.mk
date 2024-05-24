################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.c \
../Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.c \
../Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c \
../Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c 

OBJS += \
./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.o \
./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.o \
./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o \
./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o 

C_DEPS += \
./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.d \
./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.d \
./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d \
./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP/Source/BayesFunctions/%.o Drivers/CMSIS-DSP/Source/BayesFunctions/%.su Drivers/CMSIS-DSP/Source/BayesFunctions/%.cyclo: ../Drivers/CMSIS-DSP/Source/BayesFunctions/%.c Drivers/CMSIS-DSP/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Zeng/STM32CubeIDE/Workspace_Test/STM32F769_Audio_Init/Drivers/CMSIS-DSP/Include" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2f-Source-2f-BayesFunctions

clean-Drivers-2f-CMSIS-2d-DSP-2f-Source-2f-BayesFunctions:
	-$(RM) ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.cyclo ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.d ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.o ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.su ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.cyclo ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.d ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.o ./Drivers/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.su ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.cyclo ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.su ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.cyclo ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o ./Drivers/CMSIS-DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2f-Source-2f-BayesFunctions

