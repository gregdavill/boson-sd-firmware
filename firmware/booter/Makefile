BUILD_DIR?=../../

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -I$(SOC_DIRECTORY)/software/include 

OBJECTS=main.o \
		spi.o \
		crt0.o

vpath %.ld $(BOOTER_DIRECTORY)
vpath %.c $(BOOTER_DIRECTORY)
vpath %.S $(BOOTER_DIRECTORY)
vpath %.a $(PACKAGES:%=../%)

all: booter.bin
	$(PYTHON) -m litex.soc.software.memusage bios.elf $(CURDIR)/../include/generated/regions.ld $(TRIPLE)

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

booter.elf: linker.ld $(OBJECTS)

%.elf: crt0.o
	$(CC) $(LDFLAGS) \
		-T $(BOOTER_DIRECTORY)/linker.ld \
		-N -o $@ \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		
	chmod -x $@

main.o: main.c
	$(compile)

%.o: %.cpp
	$(compilexx)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) demo.elf demo.bin .*~ *~

.PHONY: all main.o clean load
