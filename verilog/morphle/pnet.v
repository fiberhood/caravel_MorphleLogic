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


// These are the building blocks for Morphle Logic, an asynchronous
// runtime reconfigurable array (ARRA).

// many signals are two bit busses
`define Vempty 0
`define V0     1
`define V1     2
// the combination 3 is not defined

// this asynchronous finite state machine is the basic building block
// of Morphle Logic. It explicitly defines 5 simple latches that
// directly change when their inputs do, so there is no clock anywhere

module rycfsm (
    input reset,
    input [1:0] in,
    input [1:0] match,
    output [1:0] out);
    
    wire [1:0] lin;
    wire [1:0] nlin;
    wire [1:0] lmatch;
    wire [1:0] nlmatch;
    wire lmempty;
    wire nlmempty;
        
    wire linval    =| lin; // lin != `Vempty;
    wire inval     =| in; // in != `Vempty;
    wire lmatchval =| lmatch; // lmatch != `Vempty;
    wire matchval  =| match; // match != `Vempty;

    wire clear;    
    assign #1 clear = reset | (lmempty & linval & ~inval);
    wire [1:0] clear2 = {clear,clear};
    
    // two bit latches
    assign #1 lin = ~(clear2 | nlin);
    assign #1 nlin = ~(in | lin);
    
    assign #1 lmatch = ~(clear2 | nlmatch);
    assign #1 nlmatch = ~((match & {nlmempty,nlmempty}) | lmatch);
    
    // one bit latch
    assign #1 lmempty = ~(~(linval | lmatchval) | nlmempty);
    assign #1 nlmempty = ~((lmatchval & ~matchval) | lmempty);
    
    // forward the result of combining match and in
    assign out[1] = lin[1] & lmatch[1];
    assign out[0] = (lmatch[1] & lin[0]) | (lmatch[0] & linval);
    
endmodule

// each pseudo "red cell" in Morphle Logic can be configured to one these
// different options.
// 0001 .
// 0010 |
// 0100 raw
// 1000 i/o


// these pseudo red cells connect a column of yellow cells to multiplexers
// that all got to the logic analyzer pins
//
// Values are indicated by a pair of wires, where 00 indicates empty, 01 a
// 0 value and 10 indicates a 1 value. The combination 11 should never appear

module prcell(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
  // control
  input reset,    // freezes the cell operations and clears everything
  input rconfclk,   // a strobe to enter the configuration bits
  input [3:0] data_in,   // talk to logic analyzer
  output [3:0] data_out,
  output vempty,  // this cell interrupts vertical signals
  output vempty2,
  // UP
  input uempty,    // cell U is empty, so we are the topmost of a signal
  input [1:0] uin,
  output [1:0] uout,
  // DOWN
  input dempty,    // cell D is empty, so we are the bottommost of a signal
  input [1:0] din,
  output [1:0] dout);
  
  reg [3:0] cnfg;
  always @(posedge rconfclk) cnfg = data_in;
    
  // configuration signals decoded
  wire empty;
  wire vblock, vbypass;
  
  assign vbypass = cnfg[3] | cnfg[1];               
  assign vempty = empty | cnfg[0];
  assign vempty2 = vempty;
  wire vreset = reset | cnfg[0];
  wire [1:0] vin;
  wire [1:0] vout;
  wire [1:0] vback;

  wire [1:0] vmatch = data_in[1:0];
  rycfsm vfsm (.reset(vreset), .in(vin), .match(vmatch), .out(vout));
  wire [1:0] bvout = vbypass ? vin : vout;
  assign dout = cnfg[2] ? data_in[1:0] : bvout;
  assign #1 vin = uempty ? {~vreset&(~(vback[1]|vback[1'b0])),1'b0} : uin; // no oscillation on reset
  assign vback = (dempty | vempty) ? bvout : din; // don't propagate when bottommost or empty
  assign uout = cnfg[2] ? data_in[3:2] : vback;
  
  assign data_out = {uin,din};

endmodule

// this block is a short row of prcells which "cap" a block of yellow cells

module prcap #(parameter
  BLOCKWIDTH = 8,
  HMSB = BLOCKWIDTH-1,
  HMSB2 = (2*BLOCKWIDTH)-1,
  HMSB4 = (4*BLOCKWIDTH)-1)
  (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
  // control
  input reset,              // freezes the cell operations and clears everything
  input rconfclk,           // a strobe to configure the pseudo red cells
  input [HMSB4:0] data_in,      // bits from logic analyzer
  output [HMSB4:0] data_out,   // bits to logic analyser
  output [HMSB:0] vempty,  // this cell interrupts vertical signals
  output [HMSB:0] vempty2,
  // UP
  input [HMSB:0] uempty,    // cell U is empty, so we are the topmost of a signal
  input [HMSB2:0] uin,
  output [HMSB2:0] uout,
  // DOWN
  input [HMSB:0] dempty,    // cell D is empty, so we are the bottommost of a signal
  input [HMSB2:0] din,
  output [HMSB2:0] dout);

  
  genvar c;
  
  generate
    for ( c = 0 ; c < BLOCKWIDTH ; c = c + 1 ) begin : column
      prcell rc (
`ifdef USE_POWER_PINS
             .vccd1(vccd1),
             .vssd1(vssd1),
`endif
             .reset(reset),
             .rconfclk(rconfclk),
             .data_in(data_in[4*c+3:4*c]),
             .data_out(data_out[4*c+3:4*c]),
             .vempty(vempty[c]),.vempty2(vempty2[c]),
             .uempty(uempty[c]),.uin(uin[2*c+1:2*c]),.uout(uout[2*c+1:2*c]),
             .dempty(dempty[c]),.din(din[2*c+1:2*c]),.dout(dout[2*c+1:2*c]));
    end
  endgenerate

endmodule
