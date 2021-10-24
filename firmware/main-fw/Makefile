include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -fpack-struct -O2 -I$(MAIN-FW_DIRECTORY)/include -fdata-sections -ffunction-sections

OBJECTS=main.o time.o # boson.o flirCRC.o Serializer_BuiltIn.o 

vpath % $(MAIN-FW_DIRECTORY) $(MAIN-FW_DIRECTORY)/boson 

all: main-fw.bin size

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.software.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.software.mkmscimg $@
endif

main-fw.elf:


main-fw.elf: crt0.o $(MAIN-FW_DIRECTORY)/linker.ld $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(MAIN-FW_DIRECTORY)/linker.ld -N -o $@ \
		crt0.o \
		$(OBJECTS) \
		$(PACKAGES:%=-L../%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-lc -llitesdcard -lcompiler_rt -lbase -lfatfs

ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

crt0.o: $(CPU_DIRECTORY)/crt0.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) main-fw.elf main-fw.bin .*~ *~

size:
	$(TARGET_PREFIX)size main-fw.elf


.PHONY: all clean
