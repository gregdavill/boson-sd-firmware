BUILD_DIR?=../../

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

CFLAGS += -Os -DCRC16_FAST -I$(MAIN_FW_BOOTSTRAP_DIRECTORY)/include

OBJECTS=main.o \
		hyperram.o 
		

vpath %.ld $(MAIN_FW_BOOTSTRAP_DIRECTORY)
vpath %.c $(MAIN_FW_BOOTSTRAP_DIRECTORY)
vpath %.S $(MAIN_FW_BOOTSTRAP_DIRECTORY)
vpath %.a $(PACKAGES:%=../%)

all: main_fw_bootstrap.bin
	$(TRIPLE)-size main_fw_bootstrap.elf

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

main_fw_bootstrap.elf: linker.ld $(OBJECTS)

%.elf: crt0.o
	$(CC) $(LDFLAGS) \
		-T $(MAIN_FW_BOOTSTRAP_DIRECTORY)/linker.ld \
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
