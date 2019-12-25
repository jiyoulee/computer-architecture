`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, pctoreg;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regdstra, regwrite;
  wire		  jump, link;
  wire [3:0]  alucontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
	.funct      (instr[5:0]), 
	.zero       (zero),
	.signext    (signext),
	.shiftl16   (shiftl16),
	.memtoreg   (memtoreg),
	.pctoreg	(pctoreg),
	.memwrite   (memwrite),
	.pcsrc      (pcsrc),
	.alusrc     (alusrc),
	.regdst     (regdst),
	.regdstra	(regdstra),
	.regwrite   (regwrite),
	.jump       (jump),
	.link		(link),
	.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
	.pctoreg	(pctoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
	.regdstra	(regdstra),
    .regwrite   (regwrite),
    .jump       (jump),
	.link		(link),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, pctoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regdstra, regwrite,
                  output       jump, link,
                  output [3:0] alucontrol);

  wire [1:0] aluop;
  wire       branch, le;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
	.pctoreg  (pctoreg),
    .memwrite (memwrite),
	.branch	  (branch),
	.le		  (le),
    .alusrc   (alusrc),
    .regdst   (regdst),
	.regdstra (regdstra),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

  assign pcsrc = (branch & zero & ~op[0]) | (branch & ~zero & op[0]);
  assign link = le & (funct == 6'b001000);

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, pctoreg, memwrite,
               output       branch, le, alusrc,
               output       regdst, regdstra, regwrite,
               output       jump,
               output [1:0] aluop);

  reg [13:0] controls;

  assign {signext, shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, memwrite,
          memtoreg, pctoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 14'b00110001000011; // Rtype
      6'b100011: controls <= #`mydelay 14'b10100100010000; // LW
      6'b101011: controls <= #`mydelay 14'b10000100100000; // SW
	  6'b000011: controls <= #`mydelay 14'b00101000001100; // JAL
      6'b000100,
	  6'b000101: controls <= #`mydelay 14'b10000010000001; // BEQ, BNE: only difference is pcsrc
      6'b001000, 
      6'b001001: controls <= #`mydelay 14'b10100100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 14'b00100100000010; // ORI
      6'b001111: controls <= #`mydelay 14'b01100100000000; // LUI
      6'b000010: controls <= #`mydelay 14'b00000000000100; // J
      default:   controls <= #`mydelay 14'bxxxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 4'b0100;  // add
      2'b01: alucontrol <= #`mydelay 4'b1100;  // sub
      2'b10: alucontrol <= #`mydelay 4'b0010;  // or
      default: case(funct)          // RTYPE
		  6'b001000: alucontrol <= #`mydelay 4'b0100; // JR
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 4'b0100; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b1100; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0010; // OR
          6'b101010: alucontrol <= #`mydelay 4'b1110; // SLT
		  6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pctoreg, pcsrc,
                input         alusrc, regdst, regdstra,
                input         regwrite, jump, link,
                input  [3:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  srcreg, writereg;
  wire [31:0] pcnext, pcnextj, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] resultd, result;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcjmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s    (jump),
    .y    (pcnextj));
	
  mux2 #(32) pcmux(
    .d0	  (pcnextj),
	.d1	  (aluout),
	.s	  (link),
	.y	  (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (regdst),
    .y   (srcreg));
	
  mux2 #(5) wrlmux( // writelinkmux
    .d0  (srcreg),
	.d1  (5'b11111),
	.s	 (regdstra),
	.y   (writereg));

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
    .y  (resultd));
	
  mux2 #(32) reslmux( //registerlinkmux
    .d0 (resultd),
	.d1 (pcplus4),
	.s  (pctoreg),
	.y  (result));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
endmodule
