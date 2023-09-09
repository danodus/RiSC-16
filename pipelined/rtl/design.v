/*
MIT License

Copyright (c) 2022 Ashwin Rajesh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

`ifndef DESIGN_SV
`define DESIGN_SV

`include "core.v"
`include "mem_data.v"

module toplevel (
    input clk,                  // Global clock
    output[15:0] pc,
    output[15:0] display
);

    localparam p_INST_NUM = 1024;
    localparam p_DATA_ADDR_LEN = 10;
    localparam p_DATA_NUM  = 2 ** p_DATA_ADDR_LEN;
    parameter p_CODE_FILE = "code.data";
    
    reg[15:0] inst_memory[p_INST_NUM-1:0];

    reg[15:0] r_display = 16'd0;
    assign display = r_display;

    initial begin
        $readmemb(p_CODE_FILE, inst_memory);
    end

    wire[15:0] w_rd_data;
    wire[15:0] w_wr_data;
    wire[15:0] w_addr;
    wire w_wr_en;

    reg[15:0] r_addr_prev;
    
    always @(posedge clk)
        r_addr_prev <= w_addr;

    // The DUT	
    core core_dut (
        .i_clk(clk),
        .i_rst(0),

        .i_inst(inst_memory[pc[$clog2(p_INST_NUM)-1:0]]),
        .o_pc_next(pc),

        .i_mem_rd_data((r_addr_prev < p_DATA_NUM) ? w_rd_data : 16'b0),
        .o_mem_wr_data(w_wr_data),
        .o_mem_addr(w_addr),
        .o_mem_wr_en(w_wr_en)
    );

    always @(posedge clk)
        if (w_addr == 16'hFFFF)
            r_display <= w_wr_data;

    mem_data #(
        .p_WORD_LEN(16),
        .p_ADDR_LEN(p_DATA_ADDR_LEN)
    ) datamem (
        .i_clk(clk),
        .i_wr_en(w_wr_en && (w_addr < p_DATA_NUM)),
        
        .i_addr(w_addr[p_DATA_ADDR_LEN-1:0]),
        .o_rd_data(w_rd_data),
        .i_wr_data(w_wr_data)
    );
    
endmodule

`endif
