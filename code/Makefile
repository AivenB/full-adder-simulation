########################################
# Project
########################################

PROJECT = 1-bit_full_adder

########################################
# Toolchain
########################################

CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

########################################
# MCU / FPU
########################################

CPU      = -mcpu=cortex-m4
FPU      = -mfpu=fpv4-sp-d16
FLOATABI = -mfloat-abi=hard
MCU      = $(CPU) -mthumb $(FPU) $(FLOATABI)

########################################
# Paths
########################################

# Source directories
SRCDIRS = Core/Src \
          Drivers/STM32F4xx_HAL_Driver/Src

# Add these if you actually use code in them
SRCDIRS += Middlewares \
           USB_HOST

# Startup file (adjust path if needed)
STARTUP = ./Core/Startup/startup_stm32f407vgtx.s

# Include directories
INCLUDES = \
  -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IMiddlewares \
  -IUSB_HOST

########################################
# Files
########################################

# Collect all C files from SRCDIRS
C_SOURCES = $(foreach dir,$(SRCDIRS),$(wildcard $(dir)/*.c))

# Objects
OBJ = $(C_SOURCES:.c=.o) $(STARTUP:.s=.o)

########################################
# Flags
########################################

DEFS = -DUSE_HAL_DRIVER -DSTM32F407xx

CFLAGS  = $(MCU) -O2 -g3 -ffunction-sections -fdata-sections -Wall -Wextra
CFLAGS += $(DEFS) $(INCLUDES)

ASFLAGS = $(MCU) -g3

LDFLAGS = $(MCU) -Wl,--gc-sections -Wl,-Map=$(PROJECT).map \
          -TSTM32F407VGTX_FLASH.ld

########################################
# Targets
########################################

all: $(PROJECT).bin

$(PROJECT).elf: $(OBJ)
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

########################################
# Flash (using STM32CubeProgrammer CLI)
########################################

flash: $(PROJECT).bin
	STM32_Programmer_CLI -c port=SWD -w $(PROJECT).bin 0x08000000 -rst

########################################
# Clean
########################################

clean:
	rm -f $(OBJ) $(PROJECT).elf $(PROJECT).bin $(PROJECT).map

.PHONY: all flash clean

