#  Project Name
EXTLIBDIR=/User/shota/sat/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16
EXTLIBDIR = .

PERIPH_PROJECT=stm32f4_periph
USB_PROJECT=stm32f4_usb
USB_HOST_PROJECT=stm32f4_usbhost
USB_OTG_PROJECT=stm32f4_usbotg

LIB_HOME=.

#  List of the objects files to be compiled/assembled

PERIPH_SOURCES:=$(wildcard $(LIB_HOME)/STM32F4xx_StdPeriph_Driver/src/*.c) 
PERIPH_OBJECTS:=$(PERIPH_SOURCES:.c=.o)
# USB_SOURCES:=$(wildcard $(LIB_HOME)/STM32_USB_Device_Library/Core/src/*.c) 
# USB_OBJECTS:=$(USB_SOURCES:.c=.o)
#USB_OTG_SOURCES:=$(filter-out $(LIB_HOME)/STM32_USB_OTG_Driver/src/usb_bsp_template.c, $(wildcard $(LIB_HOME)/STM32_USB_OTG_Driver/src/*.c))
#USB_OTG_OBJECTS:=$(USB_OTG_SOURCES:.c=.o)
# USB_HOST_SOURCES:=$(wildcard $(LIB_HOME)/STM32_USB_HOST_Library/Core/src/*.c)
# USB_HOST_OBJECTS:=$(USB_HOST_SOURCES:.c=.o)

#OPTIMIZATION = -Os -fno-schedule-insns2 -fsection-anchors -ffunction-sections -DUSE_STDPERIPH_DRIVER -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -DUSB_OTG_HS_CORE -DUSE_ULPI_PHY -DUSE_USB_OTG_HS -fomit-frame-pointer -fno-unroll-loops -D__BUFSIZ__=256 -mabi=aapcs
#DEBUG = -g
INCLUDES = -I.. -I./mruby-arm/include -I./STM32F4-Discovery_FW_V1.1.0/Project/Demonstration -I$(LIB_HOME)/CMSIS/Include -I$(LIB_HOME)/STM32F4xx_StdPeriph_Driver/inc \
-I$(LIB_HOME)/STM32_USB_Device_Library/Core/inc -I$(LIB_HOME)/STM32_USB_HOST_Library/Core/inc \
-I$(LIB_HOME)/STM32_USB_OTG_Driver/inc -I$(LIB_HOME)/CMSIS/ST/STM32F4xx/Include \
-I./STM32F4-Discovery_FW_V1.1.0/Utilities/STM32F4-Discovery/ -I./STM32F4-Discovery_FW_V1.1.0/Project/FW_upgrade/inc/

#  Compiler Options

CFLAGS = -Wall $(OPTIMIZATION) $(DEBUG)
CFLAGS+=-MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
# -march=armv7e-m -mtune=cortex-m3
CFLAGS+=-g -Os -mapcs-frame -mno-sched-prolog -std=gnu99 -gdwarf-2 -fno-strict-aliasing -fsigned-char -ffunction-sections -fdata-sections -fno-schedule-insns2 --param max-inline-insns-single=1000 -fno-common -fno-hosted -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -flto-partition=none -fipa-sra
AFLAGS  = -mcpu=cortex-m4 -mthumb -mthumb-interwork -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS  = -mcpu=cortex-m4 -mthumb -mthumb-interwork -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 \

#CFLAGS += -I./inc $(INCLUDES)
CFLAGS += -I./../../inc $(INCLUDES)
#LDFLAGS = -static -mcpu=cortex-m4 -mfloat-abi=softfp -mthumb $(OPTIMIZATION)
LDFLAGS = -static -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb $(OPTIMIZATION)
ASFLAGS = -mcpu=cortex-m4 --defsym RAM_MODE=0

MAKE_FLAGS = --no-print-directory CC='$(CC)' LL='$(LL)' ALL_CFLAGS='$(ALL_CFLAGS)'

#  Compiler/Assembler/Linker Paths
GCC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
STRIP = arm-none-eabi-strip
OBJDUMP = arm-none-eabi-objdump
REMOVE = rm -f
SIZE = arm-none-eabi-size

#########################################################################
.PHONY: all
all: $(PERIPH_OBJECTS) $(USB_OBJECTS) $(USB_OTG_OBJECTS) host_objects
	$(AR) r lib$(PERIPH_PROJECT).a $(PERIPH_OBJECTS)
#	$(AR) r lib$(USB_PROJECT).a $(USB_OBJECTS)
#	$(AR) r lib$(USB_HOST_PROJECT).a $(USB_HOST_OBJECTS)
#	$(AR) r lib$(USB_OTG_PROJECT).a $(USB_OTG_OBJECTS)

#	@$(MAKE) -C src $(MAKE_FLAGS)
#	


host_objects: $(USB_HOST_OBJECTS)

clean:
	$(REMOVE) $(PERIPH_OBJECTS) $(USB_OBJECTS) $(USB_HOST_OBJECTS) $(USB_OTG_OBJECTS)
	$(REMOVE) lib*.a

#########################################################################
#  Default rules to compile .c and .cpp file to .o
#  and assemble .s files to .o

.c.o :
	$(GCC) $(CFLAGS) -c $< -o $@

.cpp.o :
	$(GCC) $(CFLAGS) -c $<

.S.o :
	$(AS) $(ASFLAGS) -o $@ $<

#########################################################################
