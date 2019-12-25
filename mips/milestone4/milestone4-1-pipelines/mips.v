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

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
	.memwrite   (memwrite),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

//  assign pcsrc = (branch & zero & ~funct[0]) | (branch & ~zero * funct[0]);
// assign link = le & (funct == 6'b001000);

module maindec(input       [5:0]  op,
               output reg [13:0] controls);

//  assign {signext, shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, memwrite,
//          memtoreg, pctoreg, jump, aluop} = controls;

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

module pcdec(input  [5:0] funct,
			 input        branch, zero,
			 input        le,
			 output       pcsrc, link);
			 
  assign pcsrc = (branch & zero & ~funct[0]) | (branch & ~zero * funct[0]);
  assign link = le & (funct == 6'b001000);

endmodule

module datapath(input         clk, reset,
				output        memwrite,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire  [4:0]  srcreg, writereg;
  wire  [31:0] pcnext, pcnextj, pcnextbr, pcplus4, pcbranch;
  wire  [31:0] signimm, signimmsh, shiftedimm;
  wire  [31:0] srca, srcb;
  wire  [31:0] resultd, result;
  wire         shift;

  wire  [63:0] ifidd, ifidq;
  wire  [31:0] id_pcplus4, id_instr, id_writedata;
  wire  [13:0] controls;
  wire         id_signext;
  
  wire [166:0] idexd, idexq;
  wire  [31:0] ex_pcplus4, ex_srca, ex_writedata, ex_signimm, ex_pcnextj, ex_aluout;
  wire  [25:0] ex_instr;
  wire  [12:0] ex_controls;
  wire   [4:0] ex_writereg;
  wire   [3:0] alucontrol;
  wire   [1:0] ex_aluop;
  wire         ex_regdst, ex_regdstra, ex_shiftl16, ex_alusrc, ex_zero;
  
  wire [178:0] exmemd, exmemq;
  wire  [31:0] mem_pcbranch, mem_pcnextj, mem_aluout, mem_pcplus4;
  wire   [7:0] mem_controls;
  wire   [5:0] mem_funct;
  wire   [4:0] mem_writereg;
  wire         mem_branch, mem_le, mem_jump, mem_zero;
  wire         pcsrc, link;
  
  wire  [103:0] memwbd, memwbq;
  wire   [31:0] wb_aluout, wb_readdata, wb_pcplus4;
  wire    [4:0] wb_writereg;
  wire          wb_regwrite, wb_memtoreg, wb_pctoreg;
  
  assign ifidd = {pcplus4, instr};
  assign {id_pcplus4, id_instr} = ifidq;
  assign id_signext = controls[13];
  
  assign idexd = {controls[12:0], id_pcplus4, srca, id_writedata, signimm, id_instr[25:0]};
  assign {ex_controls, ex_pcplus4, ex_srca, ex_writedata, ex_signimm, ex_instr} = idexq[166:0];
  assign ex_shiftl16 = ex_controls[12];
  assign {ex_regdst, ex_regdstra, ex_alusrc} = ex_controls[10:8];
  assign ex_aluop = ex_controls[1:0];
  assign ex_pcnextj = {ex_pcplus4[31:28], ex_instr[25:0], 2'b00};
 
  assign exmemd = {ex_controls[11], ex_controls[7:2], ex_zero, ex_instr[5:0], pcbranch, ex_pcnextj, ex_aluout, ex_writedata, ex_writereg, ex_pcplus4};
  assign {mem_controls, mem_funct, mem_pcbranch, mem_pcnextj, mem_aluout, writedata, mem_writereg, mem_pcplus4} = exmemq;
  assign aluout = mem_aluout;
  assign {mem_branch, mem_le, memwrite} = mem_controls[6:4];
  assign {mem_jump, mem_zero} = mem_controls[1:0];
  
  // controls: {regwrite, memtoreg, pctoreg}
  assign memwbd = {mem_controls[7], mem_controls[3:2], mem_writereg, mem_aluout, readdata, mem_pcplus4};
  assign {wb_regwrite, wb_memtoreg, wb_pctoreg, wb_writereg, wb_aluout, wb_readdata, wb_pcplus4} = memwbq;

  // pipeline logic
  flopr #(64) IFID(
	.clk(clk),
	.reset(reset),
	.d(ifidd),
	.q(ifidq));
	
  flopr #(167) IDEX(
	.clk(clk),
	.reset(reset),
	.d(idexd),
	.q(idexq));
	
  flopr #(179) EXMEM(
	.clk(clk),
	.reset(reset),
	.d(exmemd),
	.q(exmemq));
	
  flopr #(104) MEMWB(
	.clk(clk),
	.reset(reset),
	.d(memwbd),
	.q(memwbq));
	
  // control signal logic
  maindec md(
    .op       (id_instr[31:26]),
    .controls (controls));
	
  aludec ad( 
    .funct      (ex_instr[5:0]),
    .aluop      (ex_aluop), 
    .alucontrol (alucontrol));

  pcdec  pd(
	.funct  (mem_funct),
	.branch (mem_branch),
	.zero   (mem_zero),
	.le     (mem_le),
	.pcsrc  (pcsrc),
	.link   (link));
  
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
    .a (ex_signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (ex_pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (mem_pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcjmux(
    .d0   (pcnextbr),
    .d1   (mem_pcnextj),
    .s    (mem_jump),
    .y    (pcnextj));
	
  mux2 #(32) pcmux(
    .d0	  (pcnextj),
	.d1	  (mem_aluout),
	.s	  (link),
	.y	  (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (wb_regwrite),
    .ra1     (id_instr[25:21]),
    .ra2     (id_instr[20:16]),
    .wa      (wb_writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (id_writedata));

  mux2 #(5) wrmux(
    .d0  (ex_instr[20:16]),
    .d1  (ex_instr[15:11]),
    .s   (ex_regdst),
    .y   (srcreg));
	
  mux2 #(5) wrlmux( // writelinkmux
    .d0  (srcreg),
	.d1  (5'b11111),
	.s	 (ex_regdstra),
	.y   (ex_writereg));

  mux2 #(32) resmux(
    .d0 (wb_aluout),
    .d1 (wb_readdata),
    .s  (wb_memtoreg),
    .y  (resultd));
	
  mux2 #(32) reslmux( //registerlinkmux
    .d0 (resultd),
	.d1 (wb_pcplus4),
	.s  (wb_pctoreg),
	.y  (result));

  sign_zero_ext sze(
    .a       (id_instr[15:0]),
    .signext (id_signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimmsh[31:0]),
    .shiftl16  (ex_shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (ex_writedata),
    .d1 (shiftedimm[31:0]),
    .s  (ex_alusrc),
    .y  (srcb));

  alu alu(
    .a       (ex_srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (ex_aluout),
    .zero    (ex_zero));
    
endmodule
