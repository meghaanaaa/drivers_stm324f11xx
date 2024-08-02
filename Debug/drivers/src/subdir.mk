################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f411x_gpio_driver.c \
../drivers/src/stm32f411x_i2c_driver.c \
../drivers/src/stm32f411x_spi_driver.c 

OBJS += \
./drivers/src/stm32f411x_gpio_driver.o \
./drivers/src/stm32f411x_i2c_driver.o \
./drivers/src/stm32f411x_spi_driver.o 

C_DEPS += \
./drivers/src/stm32f411x_gpio_driver.d \
./drivers/src/stm32f411x_i2c_driver.d \
./drivers/src/stm32f411x_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I../Inc -I"D:/stm32_wrkspc/myWorkspace/target/stm32f411xdriver/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f411x_gpio_driver.cyclo ./drivers/src/stm32f411x_gpio_driver.d ./drivers/src/stm32f411x_gpio_driver.o ./drivers/src/stm32f411x_gpio_driver.su ./drivers/src/stm32f411x_i2c_driver.cyclo ./drivers/src/stm32f411x_i2c_driver.d ./drivers/src/stm32f411x_i2c_driver.o ./drivers/src/stm32f411x_i2c_driver.su ./drivers/src/stm32f411x_spi_driver.cyclo ./drivers/src/stm32f411x_spi_driver.d ./drivers/src/stm32f411x_spi_driver.o ./drivers/src/stm32f411x_spi_driver.su

.PHONY: clean-drivers-2f-src

