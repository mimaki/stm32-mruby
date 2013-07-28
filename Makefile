EXECUTABLE=stm32app.elf
BIN_IMAGE=stm32app.bin
USB_LIB=lib/usb/libstm32f4_usb.a
STDP_LIB=lib/stm32/libstm32f4_periph.a
MRUBY_LIB=mruby/build/stm32f4/lib/libmruby.a

CC=arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

EXTLIBDIR = .

AFLAGS  = -Wall -mcpu=cortex-m4 -mthumb -mthumb-interwork -mlittle-endian -mfloat-abi=softfp
CFLAGS  = $(AFLAGS) -Os -L $(EXTLIBDIR)

#usb_conf.h
CFLAGS+=-DSYSCLK_FREQ_HSE=1 -DHSE_VALUE=12000000
CFLAGS+=-DUSE_USB_OTG_HS=1 -DUSE_EMBEDDED_PHY=1 -DUSB_OTG_HS_CORE=1 -DSTDIN_USART=0 -DSTDOUT_USART=0 -DSTDERR_USART=0 -D_FS_READONLY=0 -DMPU_SUBMODEL=1 -DAPP_VERSION=1

# to run from FLASH
CFLAGS+=-Wl,-T,STM32F4xx.ld

CFLAGS+=-I./inc

# stm32f4_discovery lib
CFLAGS+=-I./lib/stm32/CMSIS/Include
CFLAGS+=-I./lib/stm32/STM32F4xx_StdPeriph_Driver/inc

# USB lib
CFLAGS+=-I./lib/usb

# mruby lib
CFLAGS+=-I./mruby/include

STDP_SOURCES:=$(wildcard ./lib/stm32/STM32F4xx_StdPeriph_Driver/src/*.c)
STDP_OBJECTS:=$(STDP_SOURCES:.c=.o)

USB_DIR    := ./lib/usb
USB_EXCERPT := $(USB_DIR)/usbd_hid_core.c
USB_SOURCES:=$(filter-out $(USB_EXCERPT),$(wildcard $(USB_DIR)/*.c))
USB_OBJECTS:=$(USB_SOURCES:.c=.o)

all: $(BIN_IMAGE) 
	$(OBJDUMP) -h -S stm32app.elf > stm32app.lss

$(USB_LIB): $(USB_OBJECTS)
	$(AR) r $(USB_LIB) $(USB_OBJECTS)

$(STDP_LIB): $(STDP_OBJECTS)
	$(AR) r $(STDP_LIB) $(STDP_OBJECTS)

$(MRUBY_LIB): $(STDP_OBJECTS)
	cd mruby/; ./minirake

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32f4xx.c startup_stm32f4xx.s stm32fxxx_it.c \
	syscalls.c stm32f4_discovery.c xprintf.c rtc_support.c \
	stm324xg_eval_fsmc_sram.c stm32f4driver.c 
	$(CC) $(CFLAGS) $^ -o $@ \
	-L$(EXTLIBDIR) -L. -L./lib/stm32 -L./lib/usb -lstm32f4_usb \
	-lstm32f4_periph -lm 

usb: $(USB_LIB)
stdp: $(STDP_LIB)
mruby: $(MRUBY_LIB)

lib: $(USB_LIB) $(STDP_LIB) $(MRUBY_LIB)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(USB_OBJECTS)
	find . -name "*.o" | xargs rm -f
	find . -name "*.d" | xargs rm -f
	find . -name "*.lss" | xargs rm -f

clean-lib:
	find . -name "*.a" | xargs rm -f

.PHONY: all clean

.c.o :
	$(CC) $(CFLAGS) -c $< -o $@

.cpp.o :
	$(CC) $(CFLAGS) -c $<

.S.o :
	$(AS) $(ASFLAGS) -o $@ $<
