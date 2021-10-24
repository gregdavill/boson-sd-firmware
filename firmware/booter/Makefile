BUILD_DIR?=../../

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -Os

OBJECTS=crt0.o \
		main.o \
		spi.o
		

vpath %.ld $(BOOTER_DIRECTORY)
vpath %.c $(BOOTER_DIRECTORY)
vpath %.S $(BOOTER_DIRECTORY)
vpath %.a $(PACKAGES:%=../%)

all: booter.bin
	$(TRIPLE)-size booter.elf

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
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		
		
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
