################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/alg.c \
../Core/Src/beamforming.c \
../Core/Src/c_math.c \
../Core/Src/config.c \
../Core/Src/gccphat.c \
../Core/Src/main.c \
../Core/Src/music.c \
../Core/Src/myprintf.c \
../Core/Src/preprocessing.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/svd_one_sided_jacobi.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/vad.c 

OBJS += \
./Core/Src/alg.o \
./Core/Src/beamforming.o \
./Core/Src/c_math.o \
./Core/Src/config.o \
./Core/Src/gccphat.o \
./Core/Src/main.o \
./Core/Src/music.o \
./Core/Src/myprintf.o \
./Core/Src/preprocessing.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/svd_one_sided_jacobi.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/vad.o 

C_DEPS += \
./Core/Src/alg.d \
./Core/Src/beamforming.d \
./Core/Src/c_math.d \
./Core/Src/config.d \
./Core/Src/gccphat.d \
./Core/Src/main.d \
./Core/Src/music.d \
./Core/Src/myprintf.d \
./Core/Src/preprocessing.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/svd_one_sided_jacobi.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/vad.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/alg.cyclo ./Core/Src/alg.d ./Core/Src/alg.o ./Core/Src/alg.su ./Core/Src/beamforming.cyclo ./Core/Src/beamforming.d ./Core/Src/beamforming.o ./Core/Src/beamforming.su ./Core/Src/c_math.cyclo ./Core/Src/c_math.d ./Core/Src/c_math.o ./Core/Src/c_math.su ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/gccphat.cyclo ./Core/Src/gccphat.d ./Core/Src/gccphat.o ./Core/Src/gccphat.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/music.cyclo ./Core/Src/music.d ./Core/Src/music.o ./Core/Src/music.su ./Core/Src/myprintf.cyclo ./Core/Src/myprintf.d ./Core/Src/myprintf.o ./Core/Src/myprintf.su ./Core/Src/preprocessing.cyclo ./Core/Src/preprocessing.d ./Core/Src/preprocessing.o ./Core/Src/preprocessing.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/svd_one_sided_jacobi.cyclo ./Core/Src/svd_one_sided_jacobi.d ./Core/Src/svd_one_sided_jacobi.o ./Core/Src/svd_one_sided_jacobi.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/vad.cyclo ./Core/Src/vad.d ./Core/Src/vad.o ./Core/Src/vad.su

.PHONY: clean-Core-2f-Src

