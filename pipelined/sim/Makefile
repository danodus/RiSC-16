VERILATOR = verilator

LDFLAGS := -LDFLAGS "$(shell sdl2-config --libs)"
CFLAGS := -CFLAGS "$(shell sdl2-config --cflags)"

ASMFILE = ../../test_asm/counter/code.asm

all: sim

clean:
	rm -rf obj_dir
	rm -f code.data

sim: sim_main.cpp
	$(VERILATOR) --trace -cc --exe $(CFLAGS) $(LDFLAGS) design.v sim_main.cpp -I../rtl
	$(MAKE) -j 4 -C obj_dir -f Vdesign.mk

code.data: $(ASMFILE)
	../../utils/assembler.py $(ASMFILE) ./code.data

run: sim code.data
	obj_dir/Vdesign

.PHONY: all clean
