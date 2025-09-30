$(info $(SHELL))

filename = top
testbench_filename = rgb_cycle
pcf_file = ../common/iceBlinkPico.pcf

build:
	yosys -p "synth_ice40 -top top -json $(filename).json" $(filename).sv
	nextpnr-ice40 --up5k --package sg48 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc
	icepack $(filename).asc $(filename).bin

prog: #for sram
	dfu-util --device 1d50:6146 --alt 0 -D $(filename).bin -R

clean:
	del /s /q $(filename).blif $(filename).asc $(filename).json $(filename).bin

test:
	iverilog -g2012 -o $(testbench_filename) $(testbench_filename)_tb.sv && vvp $(testbench_filename) && gtkwave $(testbench_filename).vcd wave_style.gtkw

show:
	gtkwave $(testbench_filename).vcd wave_style.gtkw