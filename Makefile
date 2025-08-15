# Project name
PROJECT = LR1121.v4

# Build directory
BUILD_DIR = build

# MCU settings
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# C defines
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32L476xx

# C includes
C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32L4xx_HAL_Driver/Inc \
-IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/lr11xx_driver/src

# C sources
C_SOURCES = \
Core/Src/main.c \
Core/Src/lora_base.c \
Core/Src/lora_simple_beacon.c \
Core/Src/lr11xx_hal.c \
Core/Src/stm32l4xx_hal_msp.c \
Core/Src/stm32l4xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32l4xx.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c

# Assembly sources
ASM_SOURCES = \
Core/Startup/startup_stm32l476rgtx.s

# Check what LR11xx driver sources exist
LR11XX_SOURCES = \
Drivers/lr11xx_driver/src/lr11xx_bootloader.c \
Drivers/lr11xx_driver/src/lr11xx_crypto_engine.c \
Drivers/lr11xx_driver/src/lr11xx_driver_version.c \
Drivers/lr11xx_driver/src/lr11xx_gnss.c \
Drivers/lr11xx_driver/src/lr11xx_lr_fhss.c \
Drivers/lr11xx_driver/src/lr11xx_radio_timings.c \
Drivers/lr11xx_driver/src/lr11xx_radio.c \
Drivers/lr11xx_driver/src/lr11xx_regmem.c \
Drivers/lr11xx_driver/src/lr11xx_rttof.c \
Drivers/lr11xx_driver/src/lr11xx_system.c \
Drivers/lr11xx_driver/src/lr11xx_wifi.c

# Add LR11xx sources to C sources
C_SOURCES += $(LR11XX_SOURCES)

# Linker script
LDSCRIPT = STM32L476RGTX_FLASH.ld

# Libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref -Wl,--gc-sections

# Compiler
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

# Compile flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Debug build?
DEBUG = 1
ifeq ($(DEBUG), 1)
OPT = -Og -g3
C_DEFS += -DDEBUG
else
OPT = -Os
endif

# Objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# Assembly objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Default target
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin

# Build the application
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary -S $< $@
	
$(BUILD_DIR):
	mkdir $@

# Clean
clean:
	-rm -fR $(BUILD_DIR)

# Dependencies
-include $(wildcard $(BUILD_DIR)/*.d)

# Phony targets
.PHONY: all clean

# Show make variables (for debugging)
debug:
	@echo "PROJECT: $(PROJECT)"
	@echo "BUILD_DIR: $(BUILD_DIR)"
	@echo "MCU: $(MCU)"
	@echo "C_SOURCES: $(C_SOURCES)"
	@echo "OBJECTS: $(OBJECTS)"
