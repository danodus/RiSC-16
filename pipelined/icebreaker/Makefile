YOSYS := yosys
ICEPACK := icepack
NEXTPNR := nextpnr-ice40
ICEPROG := iceprog

DEVICE := up5k
PACKAGE := sg48

PIN_DEF := icebreaker.pcf
SRC := 	../rtl/alu.v \
		../rtl/core.v \
		../rtl/design.v \
		../rtl/mem_data.v \
		../rtl/mem_reg.v \
		top.sv

DEFINES :=

ASMFILE = ../../test_asm/counter/code.asm

all: top.bin

clean:
	rm -f *.asc *.json *.bin *.log
	rm -f code.data

code.data: $(ASMFILE)
	../../utils/assembler.py $(ASMFILE) ./code.data

top.json: $(SRC) code.data
	$(YOSYS) -ql top.log -p 'verilog_defines $(DEFINES) ; read_verilog -sv $(SRC); synth_ice40 -dsp -top top -json top.json' $(SRC)

top.asc: top.json $(PIN_DEF)
	$(NEXTPNR) -l top_nextpnr.log --$(DEVICE) --package $(PACKAGE) --json top.json --pcf $(PIN_DEF) --asc top.asc --randomize-seed

top.bin: top.asc
	$(ICEPACK) -s top.asc top.bin

prog: top.bin
	$(ICEPROG) top.bin

.PHONY: all prog
