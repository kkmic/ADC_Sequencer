#
# Do a Makefile build
#

CC      = arm-none-eabi-gcc
ASM     = arm-none-eabi-as
LINK    = arm-none-eabi-gcc
SIZE     = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

TARGET_NAME = sequencer

TARGET_ELF = $(TARGET_NAME).elf
TARGET_BIN = $(TARGET_NAME).bin

SRCDIR = src

C_SOURCES = src/main.c \
            src/stm32l1xx_it.c \
            src/system_stm32l1xx.c \
            src/stm32l1xx_hal_msp.c

ASM_SOURCES = src/startup_stm32l152xe.s

EXTERNAL_SOURCES = Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c \
		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c \
		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c \
           Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c \
		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc.c \
		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc_ex.c \
		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dma.c \
		   Drivers/BSP/STM32L152C-Discovery/stm32l152c_discovery.c

#          Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_tim.c \
#		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr_ex.c 
#		   Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc_ex.c \

DIRLIST = Drivers/STM32L1xx_HAL_Driver/Src \
          Drivers/BSP/STM32L152C-Discovery

OBJDIR = .obj

DEBUG = -Os -Wall
CPU_PARM = -mcpu=cortex-m3 -mthumb -DSTM32L152xB

FIRMWARE_ROOT = ../STM32Cube_FW_L1_V1.8.0

LINKER_SCRIPT = STM32L152RB_FLASH.ld

DISCOVERY_SCRIPT = /usr/share/openocd/scripts/board/stm32ldiscovery.cfg

INCLUDES = -Iinclude \
	   -I$(FIRMWARE_ROOT)/Drivers/STM32L1xx_HAL_Driver/Inc/ \
	   -I$(FIRMWARE_ROOT)/Drivers/CMSIS/Include \
	   -I$(FIRMWARE_ROOT)/Drivers/CMSIS/Device/ST/STM32L1xx/Include \
	   -I$(FIRMWARE_ROOT)/Drivers/BSP/STM32L152C-Discovery 

CFLAGS = -fno-common \
	 -fno-exceptions \
	 -fdata-sections \
	 -ffunction-sections \
	 $(CPU_PARM) \
	 $(DEBUG) 

LDFLAGS = -fno-exceptions $(CPU_PARM) -T $(LINKER_SCRIPT) -nostdlib -Wl,-gc-sections

#Following is the flags if we use  __libc_init_array call in startup_stm32l152xb.s
#LDFLAGS = -fno-exceptions $(CPU_PARM) -T $(LINKER_SCRIPT) -Wl,-gc-sections

OBJECTS = $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(C_SOURCES)) \
          $(patsubst $(SRCDIR)/%.s,$(OBJDIR)/%.o,$(ASM_SOURCES)) \
	  $(patsubst %.c,$(OBJDIR)/%.o,$(EXTERNAL_SOURCES))

.PHONY: default clean

default: $(TARGET_BIN)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR)/%.o: $(FIRMWARE_ROOT)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.s
	$(ASM) -c $< -o $@

$(OBJECTS): | $(OBJDIR)

$(OBJDIR):
	mkdir -p $(OBJDIR) $(addprefix $(OBJDIR)/,$(DIRLIST))

$(TARGET_ELF): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

clean:
	find . -name *~ -exec rm -f {} ';'
	rm -rf $(OBJDIR)
	rm -f $(TARGET_BIN) $(TARGET_ELF)

size: $(TARGET_BIN)
	$(SIZE) $(TARGET_ELF)
	
ocd:	
	openocd -f $(DISCOVERY_SCRIPT)

pgm: $(TARGET_BIN)
	openocd -f $(DISCOVERY_SCRIPT) -c "program $(TARGET_BIN) 0x08000000 reset"
