BUILD_DIR?=../../

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -Os -DCRC16_FAST

OBJECTS=main.o \
		boot-helper.o \
		spi.o \
		hyperram.o \
		time.o \
		flash.o \
		boot.o
		

vpath %.ld $(BOOTLOADER_DIRECTORY)
vpath %.c $(BOOTLOADER_DIRECTORY)
vpath %.S $(BOOTLOADER_DIRECTORY)
vpath %.a $(PACKAGES:%=../%)

all: bootloader.bin
	$(TRIPLE)-size bootloader.elf

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.software.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.software.mkmscimg $@
endif

bootloader.elf: linker.ld $(OBJECTS)

%.elf: crt0.o
	$(CC) $(LDFLAGS) \
		-T $(BOOTLOADER_DIRECTORY)/linker.ld \
		-N -o $@ \
		crt0.o \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-lc -lcompiler_rt -lbase
		
		
	chmod -x $@

main.o: main.c
	$(compile)

crt0.o: $(CPU_DIRECTORY)/crt0.S
	$(assemble)

%.o: %.cpp
	$(compilexx)

%.o: %.c
	$(compile)


%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) demo.elf demo.bin .*~ *~

.PHONY: all main.o clean load
