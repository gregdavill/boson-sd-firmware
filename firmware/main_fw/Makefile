include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -O2 -I$(MAIN_FW_DIRECTORY)/include -I$(MAIN_FW_DIRECTORY)/../deps/fatfs/source -fdata-sections -ffunction-sections -DSDCARD_DEBUG

OBJECTS=main.o \
		timer.o \
		logger.o \
		ff.o \
		ffunicode.o \
		sdcard.o \
		cache.o \
		rb_tree.o \
		boson.o \
		flirCRC.o \
		ReturnCodes.o \
		FunctionCodes.o \
		Serializer_BuiltIn.o 

vpath % $(MAIN_FW_DIRECTORY) $(MAIN_FW_DIRECTORY)/boson $(MAIN_FW_DIRECTORY)/../deps/fatfs/source

all: main_fw.bin size

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

main_fw.elf:


main_fw.elf: crt0.o $(MAIN_FW_DIRECTORY)/linker.ld $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(MAIN_FW_DIRECTORY)/linker.ld -N -o $@ \
		crt0.o \
		$(OBJECTS) \
		$(PACKAGES:%=-L../%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-lc -lcompiler_rt -lbase

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
	$(RM) $(OBJECTS) main_fw.elf main_fw.bin .*~ *~

size:
	$(TARGET_PREFIX)size main_fw.elf


.PHONY: all clean
