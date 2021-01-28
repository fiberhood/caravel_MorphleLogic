// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// SPDX-FileCopyrightText: Copyright 2020 Jecel Mattos de Assumpcao Jr
// 
// SPDX-License-Identifier: Apache-2.0 
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is has been replaced by a block of Morphle Logic cells
 * connected to the Logic Analyzer pins
 *
 *-------------------------------------------------------------
 */

module user_proj_example (
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output reg wbs_ack_o,
    output reg [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb
);

  assign io_out = {`MPRJ_IO_PADS{1'b0}};
  assign io_oeb = {`MPRJ_IO_PADS{1'b0}};

// dummy wishbone: you read back what you write

    reg [31:0] store;

    wire valid = wbs_cyc_i & wbs_stb_i;

    always @(posedge wb_clk_i) begin
        if (wb_rst_i == 1'b 1) begin
            wbs_ack_o <= 1'b 0;
        end else begin
            if (wbs_we_i == 1'b 1) begin
                if (wbs_sel_i[0]) store[7:0]   <= wbs_dat_i[7:0];
                if (wbs_sel_i[1]) store[15:8]  <= wbs_dat_i[15:8];
                if (wbs_sel_i[2]) store[23:16] <= wbs_dat_i[23:16];
                if (wbs_sel_i[3]) store[31:24] <= wbs_dat_i[31:24];
            end
            wbs_dat_o <= store;
            wbs_ack_o <= valid & !wbs_ack_o;
        end
    end

// logic analyzer connections and test project
//
// to make things easier for the RISC-V we configure
// 127 to 64 as outputs to the circuit under test
// 63 to 0 as inputs observing points in the circuit
  
    assign la_data_out[127:32] = {{96{1'b0}}};
    
    pnarray #(.NUMROWS(4), .NUMCOLS(4), .BLOCKWIDTH(8), .BLOCKHEIGHT(8))
        blk (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .enable(la_data_in[16+96]),
             .raddr(la_data_in[15+96:8+96]),
             .caddr(la_data_in[7+96:0+96]),
             .reset(la_data_in[26+96]),
             .confclk(la_data_in[25+96]),
             .rconfclk(la_data_in[24+96]),
             .data_in(la_data_in[31+64:0+64]),
             .data_out(la_data_out[31:0])
            );

endmodule

module pnarray #(parameter
  BLOCKWIDTH = 8,
  BLOCKHEIGHT = 8,
  NUMROWS = 8,
  NUMCOLS = 16,
  HMSB = BLOCKWIDTH-1,
  HMSB4 = (4*BLOCKWIDTH)-1,
  WIDTH = BLOCKWIDTH*NUMCOLS,
  WMSB = WIDTH-1,
  WMSB2 = (2*WIDTH)-1)
  (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
  input enable,     // all other inputs are ignored if this is false
  input [7:0] raddr,
  input [7:0] caddr,
  // control
  input reset,              // freezes the cell operations and clears everything
  input confclk,            // a strobe to enter four configuration bits
  input rconfclk,           // a strobe to configure the pseudo red cells
  input [HMSB4:0] data_in,      // bits from logic analyzer
  output [HMSB4:0] data_out);   // bits to logic analyser
  
  wire [NUMROWS-1:0] rsel;
  assign rsel = {{(NUMROWS-1){1'b0}},enable} << raddr;
  
  wire [4*NUMROWS*BLOCKWIDTH-1:0] muxd;
  assign data_out = muxd >> ( raddr * BLOCKWIDTH );
  
  // bottom             top
  // \/                   \/
  // ----[2]----[1]----[0]----
  //  3      2      1      0     NUMROWS+1

  wire [WMSB:0] ve[NUMROWS:0];
  wire [WMSB:0] ve2[NUMROWS:0];
  wire [WMSB2:0] vs[NUMROWS:0]; // signal pairs
  wire [WMSB2:0] vb[NUMROWS:0]; // signal pairs back
  genvar r;
  
  generate
    for ( r = 0 ; r < NUMROWS ; r = r + 1 ) begin : line
      pnrow #(.NUMCOLS(NUMCOLS), .BLOCKWIDTH(BLOCKWIDTH), .BLOCKHEIGHT(BLOCKHEIGHT))
          pr (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .enable(rsel[r]),
             .caddr(caddr),
             .reset(reset),
             .confclk(confclk),
             .rconfclk(rconfclk),
             .data_in(data_in),
             .data_out(muxd[4*(r+1)*BLOCKWIDTH-1:4*r*BLOCKWIDTH]),
             .uvempty(ve[r]),
             .dvempty(ve2[r+1]),
             .uempty(ve2[r]),
             .uin(vs[r]),
             .uout(vb[r]),
             .dempty(ve[r+1]),
             .din(vb[r+1]),
             .dout(vs[r+1])
             );
    end
  endgenerate
  
  assign ve2[0] = {{WIDTH{1'b1}}};
  assign ve[NUMROWS] = {{WIDTH{1'b1}}};
  assign vs[0] = {{2*WIDTH{1'b0}}};
  assign vb[NUMROWS] = {{2*WIDTH{1'b0}}};
  
endmodule

module pnrow #(parameter
  BLOCKWIDTH = 8,
  BLOCKHEIGHT = 8,
  NUMCOLS = 16,
  VMSB = BLOCKHEIGHT-1,
  VMSB2 = (2*BLOCKHEIGHT)-1,
  HMSB = BLOCKWIDTH-1,
  HMSB4 = (4*BLOCKWIDTH)-1,
  WIDTH = BLOCKWIDTH*NUMCOLS,
  WMSB = WIDTH-1,
  WMSB2 = (2*WIDTH)-1)
  (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
  input enable,     // all other inputs are ignored if this is false
  input [7:0] caddr,
  // control
  input reset,              // freezes the cell operations and clears everything
  input confclk,            // a strobe to enter four configuration bits
  input rconfclk,           // a strobe to configure the pseudo red cells
  input [HMSB4:0] data_in,      // bits from logic analyzer
  output [HMSB4:0] data_out,  // bits to logic analyser
  output [WMSB:0] uvempty,  // these cells interrupt vertical signals to up
  output [WMSB:0] dvempty,  // these cells interrupt vertical signals to down
  // UP
  input [WMSB:0] uempty,    // cells U is empty, so we are the topmost of a signal
  input [WMSB2:0] uin,
  output [WMSB2:0] uout,
  // DOWN
  input [WMSB:0] dempty,    // cells D is empty, so we are the bottommost of a signal
  input [WMSB2:0] din,
  output [WMSB2:0] dout);
  
  wire [NUMCOLS-1:0] csel;
  assign csel = {{(NUMCOLS-1){1'b0}},enable} << caddr;
  
  wire [4*WIDTH-1:0] muxd;
  assign data_out = muxd >> ( caddr * BLOCKWIDTH );

  // left               right
  // \/                   \/
  // ----[2]----[1]----[0]----
  //  3      2      1      0     NUMCOLS+1

  wire [VMSB:0] he[NUMCOLS:0];
  wire [VMSB:0] he2[NUMCOLS:0];
  wire [VMSB2:0] hs[NUMCOLS:0]; // signal pairs
  wire [VMSB2:0] hb[NUMCOLS:0]; // signal pairs back
  
  genvar c;
  
  generate
    for ( c = 0 ; c < NUMCOLS ; c = c + 1 ) begin : col
      pncol #(.BLOCKWIDTH(BLOCKWIDTH), .BLOCKHEIGHT(BLOCKHEIGHT))
          pc (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .enable(csel[c]),
             .reset(reset),
             .confclk(confclk),
             .rconfclk(rconfclk),
             .data_in(data_in),
             .data_out(muxd[4*(c+1)*BLOCKWIDTH-1:4*c*BLOCKWIDTH]),
             .lhempty(he[c+1]),
             .uvempty(uvempty[(c+1)*BLOCKWIDTH-1:c*BLOCKWIDTH]),
             .rhempty(he2[c]),
             .dvempty(dvempty[(c+1)*BLOCKWIDTH-1:c*BLOCKWIDTH]),
             .uempty(uempty[(c+1)*BLOCKWIDTH-1:c*BLOCKWIDTH]),
             .uin(uin[(c+1)*2*BLOCKWIDTH-1:c*2*BLOCKWIDTH]),
             .uout(uout[(c+1)*2*BLOCKWIDTH-1:c*2*BLOCKWIDTH]),
             .dempty(dempty[(c+1)*BLOCKWIDTH-1:c*BLOCKWIDTH]),
             .din(din[(c+1)*2*BLOCKWIDTH-1:c*2*BLOCKWIDTH]),
             .dout(dout[(c+1)*2*BLOCKWIDTH-1:c*2*BLOCKWIDTH]),
             .lempty(he2[c+1]),
             .lin(hs[c+1]),
             .lout(hb[c+1]),
             .rempty(he[c]),
             .rin(hb[c]),
             .rout(hs[c])
             );
    end
  endgenerate

  assign hb[0] = {{2*BLOCKHEIGHT{1'b0}}};
  assign hs[NUMCOLS] = {{2*BLOCKHEIGHT{1'b0}}};
  assign he[0] = {{BLOCKHEIGHT{1'b1}}};
  assign he2[NUMCOLS] = {{BLOCKHEIGHT{1'b1}}};
  
endmodule

module pncol #(parameter
  BLOCKWIDTH = 8,
  BLOCKHEIGHT = 8,
  HMSB = BLOCKWIDTH-1,
  HMSB2 = (2*BLOCKWIDTH)-1,
  HMSB4 = (4*BLOCKWIDTH)-1,
  VMSB = BLOCKHEIGHT-1,
  VMSB2 = (2*BLOCKHEIGHT)-1)
  (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
  input enable,     // all other inputs are ignored if this is false
  // control
  input reset,              // freezes the cell operations and clears everything
  input confclk,            // a strobe to enter four configuration bits
  input rconfclk,           // a strobe to configure the pseudo red cells
  input [HMSB4:0] data_in,      // bits from logic analyzer
  output [HMSB4:0] data_out,   // bits to logic analyser
  output [VMSB:0] lhempty,  // this cell interrupts horizontal signals to left
  output [HMSB:0] uvempty,  // this cell interrupts vertical signals to up
  output [VMSB:0] rhempty,  // this cell interrupts horizontal signals to right
  output [HMSB:0] dvempty,  // this cell interrupts vertical signals to down
  // UP
  input [HMSB:0] uempty,    // cells U is empty, so we are the topmost of a signal
  input [HMSB2:0] uin,
  output [HMSB2:0] uout,
  // DOWN
  input [HMSB:0] dempty,    // cells D is empty, so we are the bottommost of a signal
  input [HMSB2:0] din,
  output [HMSB2:0] dout,
  // LEFT
  input [VMSB:0] lempty,    // cells L is empty, so we are the leftmost of a signal
  input [VMSB2:0] lin,
  output [VMSB2:0] lout,
  // RIGHT
  input [VMSB:0] rempty,    // cells D is empty, so we are the rightmost of a signal
  input [VMSB2:0] rin,
  output [VMSB2:0] rout);

  // latched signals stay put when logic analyser has enabled some other block
  
  reg lreset;
  //always @(enable, reset) if (enable) lreset = reset;
  assign lreset = (enable & reset) | (~enable & lreset) | (reset & lreset); // last term eliminates glitches
  reg lconfclk;
  //always @(enable, confclk) if (enable) lconfclk = confclk;
  assign lconfclk = (enable & confclk) | (~enable & lconfclk) | (confclk & lconfclk);
  reg lrconfclk;
  //always @(enable, rconfclk) if (enable) lrconfclk = rconfclk;
  assign lrconfclk = (enable & rconfclk) | (~enable & lrconfclk) | (rconfclk & lrconfclk);
  reg [HMSB4:0] ldata_in;
  wire [HMSB4:0] wenable = {{(4*BLOCKWIDTH){enable}}};
  //always @(enable, data_in) if (enable) ldata_in = data_in;
  assign ldata_in = (wenable & data_in) | (~wenable & ldata_in) | (data_in & ldata_in);
  
  wire [HMSB:0] ve;
  wire [HMSB:0] ve2;
  wire [HMSB2:0] vs;
  wire [HMSB2:0] vb;
  
  prcap #(.BLOCKWIDTH(BLOCKWIDTH))
        cap (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .reset(reset), .rconfclk(rconfclk),
             .data_in(ldata_in), .data_out(data_out),
             .vempty(uvempty),
             .vempty2(ve2),
             .uempty(uempty), .uin(uin), .uout(uout),
             .dempty(ve), .din(vb), .dout(vs)
            );
  
  wire [HMSB4:0] cbitout; // will be discarded
            
  yblock #(.BLOCKWIDTH(BLOCKWIDTH), .BLOCKHEIGHT(BLOCKHEIGHT))
        blk (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .reset(lreset), .confclk(lconfclk), .cbitin(ldata_in), .cbitout(cbitout),
             .lhempty(lhempty), .uvempty(ve),
             .rhempty(rhempty), .dvempty(dvempty),
             .uempty(ve2), .uin(vs), .uout(vb),
             .dempty(dempty), .din(din), .dout(dout),
             .lempty(lempty), .lin(lin), .lout(lout),
             .rempty(rempty), .rin(rin), .rout(rout)
            );

endmodule
