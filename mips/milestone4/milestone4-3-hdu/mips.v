`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

  // ###### Jiyou Lee: Start-End: 1, 2, 3, 4, 5, 6 ####### 

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  // ###### Jiyou Lee: Start 1 ####### 
  wire       stall, memread;
  wire [5:0] fwdcontrols;
  wire [4:0] ex_writereg, mem_writereg, wb_writereg;
  wire [4:0] ex_rs, ex_rt, id_rs, id_rt;
  wire       mem_regwrite, wb_regwrite;

  // Instantiate Forwarding Unit
  forwardingunit fdu(
    .mem_writereg    (mem_writereg),
    .wb_writereg     (wb_writereg),
    .ex_rs           (ex_rs),
    .ex_rt           (ex_rt),
    .id_rs           (id_rs),
    .id_rt           (id_rt),
    .mem_regwrite    (mem_regwrite),
    .wb_regwrite     (wb_regwrite),
    .fwdcontrols     (fwdcontrols));
	
  // Instantiate Hazard Detection Unit
  hazarddetectionunit hdu(
    .memread     (memread),
    .ex_writereg (ex_writereg),
	.id_rs       (id_rs),
	.id_rt       (id_rt),
    .stall       (stall));
  // ###### Jiyou Lee: End 1 ####### 

  // Instantiate Datapath
  datapath dp(
    .clk           (clk),
    .reset         (reset),
  // ###### Jiyou Lee: Start 2 ####### 
	.stall         (stall),
	.fwdcontrols   (fwdcontrols),
	.memread       (memread),
	.memwrite      (memwrite),
	.mem_regwrite  (mem_regwrite),
	.wb_regwrite   (wb_regwrite),
	.ex_writereg   (ex_writereg),
	.mem_writereg  (mem_writereg),
    .wb_writereg   (wb_writereg),
    .ex_rs         (ex_rs),
    .ex_rt         (ex_rt),
    .id_rs         (id_rs),
    .id_rt         (id_rt),
  // ###### Jiyou Lee: End 2 ####### 
    .pc            (pc),
    .instr         (instr),
    .aluout        (memaddr), 
    .writedata     (memwritedata),
    .readdata      (memreaddata));

endmodule

  // ###### Jiyou Lee: Start 3 ####### 
module forwardingunit(input      [4:0] mem_writereg, wb_writereg,
					  input      [4:0] ex_rs, ex_rt, id_rs, id_rt,
					  input            mem_regwrite, wb_regwrite,
					  output reg [5:0] fwdcontrols);
  
  wire [1:0] fducontrols = {mem_regwrite, wb_regwrite};

  always @(*)
    case(fducontrols[1:0])
	  2'b00:   fwdcontrols <= #`mydelay 6'b000000;
	  2'b01:   begin
	           fwdcontrols[5:4] <= #`mydelay ((wb_writereg == ex_rs) && (wb_writereg != 0)) ? 2'b10 : 2'b00;
	           fwdcontrols[3:2] <= #`mydelay ((wb_writereg == ex_rt) && (wb_writereg != 0)) ? 2'b10 : 2'b00;
			   fwdcontrols[1]   <= #`mydelay ((wb_writereg == id_rs) && (wb_writereg != 0)) ?  1'b1 : 1'b0;
			   fwdcontrols[0]   <= #`mydelay ((wb_writereg == id_rt) && (wb_writereg != 0)) ? 1'b1 : 1'b0;
			   end
	  2'b10:   begin
			   fwdcontrols[5:4] <= #`mydelay ((mem_writereg == ex_rs) && (mem_writereg != 0)) ? 2'b01 : 2'b00;
	           fwdcontrols[3:2] <= #`mydelay ((mem_writereg == ex_rt) && (mem_writereg != 0)) ? 2'b01 : 2'b00;
			   fwdcontrols[1]   <= #`mydelay 1'b0;
			   fwdcontrols[0]   <= #`mydelay 1'b0;
			   end
	  2'b11:   begin
			   fwdcontrols[5:4] <= #`mydelay ((mem_writereg == ex_rs) && (mem_writereg != 0)) ? 2'b01 : (((wb_writereg == ex_rs) && (wb_writereg != 0)) ? 2'b10 : 2'b00);
	           fwdcontrols[3:2] <= #`mydelay ((mem_writereg == ex_rt) && (mem_writereg != 0)) ? 2'b01 : (((wb_writereg == ex_rt) && (wb_writereg != 0)) ? 2'b10 : 2'b00);
			   fwdcontrols[1]   <= #`mydelay ((wb_writereg == id_rs) && (wb_writereg != 0)) ? 1'b1 : 1'b0;
			   fwdcontrols[0]   <= #`mydelay ((wb_writereg == id_rt) && (wb_writereg != 0)) ? 1'b1 : 1'b0;
			   end
	endcase
					  
endmodule


module hazarddetectionunit(input        memread,
						   input  [4:0] ex_writereg,
                           input  [4:0] id_rs, id_rt,
                           output       stall);
	
	assign stall = (memread) && ((ex_writereg == id_rs) || (ex_writereg == id_rt));
	
endmodule
  // ###### Jiyou Lee: End 3 ####### 

module datapath(input         clk, reset,
  // ###### Jiyou Lee: Start 4 ####### 
				input         stall,
				input   [5:0] fwdcontrols,
				output        memread, memwrite,
				output        mem_regwrite, wb_regwrite,
				output  [4:0] ex_writereg, mem_writereg, wb_writereg,
				output  [4:0] ex_rs, ex_rt, id_rs, id_rt,
  // ###### Jiyou Lee: End 4 ####### 
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire  [4:0]  srcreg, writereg;
  wire  [31:0] pcnext, pcnextj, pcnextbr, pcplus4, pcbranch;
  wire  [31:0] signimm, shiftedimm;
  wire  [31:0] srca, srcb;
  wire  [31:0] resultd, result;
  wire         shift;

  // ###### Jiyou Lee: Start 5 ####### 
  // pipelining declarations
  // cf) some wires are to be used in Milestone 5
  wire  [63:0] ifidd, ifidq;
  wire  [31:0] id_pcplus4, id_instr, id_pcnextj, id_writedata;
  wire  [14:0] controls;
  wire         id_regwrite, id_memwrite, id_memread, id_signext, id_jump;
  
  wire [167:0] idexd, idexq;
  wire  [31:0] ex_pcplus4, ex_rd1, ex_rd2, ex_signimm, ex_aluout, ex_writedata;
  wire  [25:0] ex_instr;
  wire  [13:0] ex_controls;
  wire   [3:0] alucontrol;
  wire   [1:0] ex_aluop;
  wire         ex_regdst, ex_regdstra, ex_shiftl16, ex_alusrc, ex_zero;
  
  wire [114:0] exmemd, exmemq;
  wire  [31:0] mem_aluout, mem_pcplus4;
  wire   [7:0] mem_controls;
  wire   [5:0] mem_funct;
  wire         mem_branch, mem_le, mem_zero;
  wire         pcsrc, link;
  
  wire  [103:0] memwbd, memwbq;
  wire   [31:0] wb_aluout, wb_readdata, wb_pcplus4;
  wire          wb_memtoreg, wb_pctoreg;
  
  // forwarding declarations
  wire   [31:0] fwd_srca, fwd_srcb;
  wire   [31:0] fwd_rd1, fwd_rd2;
  
  // pipelining assignments
  // controls: {id_signext, shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, memread, memwrite, memtoreg, pctoreg, jump, aluop}
  assign ifidd = {pcplus4, instr};
  assign {id_pcplus4, id_instr} = ifidq;
  assign id_signext = controls[14];
  assign id_jump = controls[2];
  assign id_pcnextj = {id_pcplus4[31:28], id_instr[25:0], 2'b00};
  
  // controls: {shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, memread, memwrite, memtoreg, pctoreg, jump, aluop}
  assign idexd = {controls[13], id_regwrite, controls[11:7], id_memread, id_memwrite, controls[4:0], id_pcplus4, fwd_rd1, fwd_rd2, signimm, id_instr[25:0]};
  assign {ex_controls, ex_pcplus4, ex_rd1, ex_rd2, ex_signimm, ex_instr} = idexq;
  assign ex_shiftl16 = ex_controls[13];
  assign memread = ex_controls[6];
  assign {ex_regdst, ex_regdstra, ex_alusrc} = ex_controls[11:9];
  assign ex_aluop = ex_controls[1:0];

  // controls: {regwrite, branch, le, memwrite, memtoreg, pctoreg, jump, zero} 
  assign exmemd = {ex_controls[12], ex_controls[8:7], ex_controls[5:2], ex_zero, ex_instr[5:0], ex_aluout, ex_writedata, ex_writereg, ex_pcplus4};
  assign {mem_controls, mem_funct, mem_aluout, writedata, mem_writereg, mem_pcplus4} = exmemq;
  assign aluout = mem_aluout;
  assign {mem_regwrite, mem_branch, mem_le, memwrite} = mem_controls[7:4];
  assign mem_zero = mem_controls[0];
  
  // controls: {regwrite, memtoreg, pctoreg}
  assign memwbd = {mem_controls[7], mem_controls[3:2], mem_writereg, mem_aluout, readdata, mem_pcplus4};
  assign {wb_regwrite, wb_memtoreg, wb_pctoreg, wb_writereg, wb_aluout, wb_readdata, wb_pcplus4} = memwbq;

  // forwarding assignments
  assign id_rs = id_instr[25:21];
  assign id_rt = id_instr[20:16];
  
  assign ex_rs = ex_instr[25:21];
  assign ex_rt = ex_instr[20:16];

  // pipelining logic
  flopenr #(64) IFID(
	.clk   (clk),
	.reset (reset),
	.en    (~stall),
	.d     (ifidd),
	.q     (ifidq));
	
  flopr #(168) IDEX(
	.clk   (clk),
	.reset (reset),
	.d     (idexd),
	.q     (idexq));
	
  flopr #(115) EXMEM(
	.clk   (clk),
	.reset (reset),
	.d     (exmemd),
	.q     (exmemq));
	
  flopr #(104) MEMWB(
	.clk   (clk),
	.reset (reset),
	.d     (memwbd),
	.q     (memwbq));
	
  // hazard resolution logic
  mux2 #(1) mwctrlmux (
    .d0 (controls[5]),
	.d1 (1'b0),
	.s  (stall),
	.y  (id_memwrite));
  
  mux2 #(1) rwctrlmux (
    .d0 (controls[12]),
	.d1 (1'b0),
	.s  (stall),
	.y  (id_regwrite));
	
  mux2 #(1) mrctrlmux (
    .d0 (controls[6]),
	.d1 (1'b0),
	.s  (stall),
	.y  (id_memread));
	
  // control signal logic
  maindec md(
    .op       (id_instr[31:26]),
    .controls (controls));
	
  aludec ad( 
    .funct      (ex_instr[5:0]),
    .aluop      (ex_aluop), 
    .alucontrol (alucontrol));

  pcdec  pd( // cf) instance to be used in Milestone 5
	.funct  (mem_funct),
	.branch (mem_branch),
	.zero   (mem_zero),
	.le     (mem_le),
	.pcsrc  (pcsrc),
	.link   (link));
  
  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	.en    (~stall),
    .d     (pcnext),
    .q     (pc));
  // ###### Jiyou Lee: End 5 ####### 

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  // ###### Jiyou Lee: Start 6 ####### 
  mux2 #(32) pcjmux(
    .d0   (pcplus4),
    .d1   (id_pcnextj),
    .s    (id_jump),
    .y    (pcnext));
	
  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (wb_regwrite),
    .ra1     (id_rs),
    .ra2     (id_rt),
    .wa      (wb_writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (id_writedata));
	
  mux2 #(32) rd1fwdmux(
    .d0  (srca),
	.d1  (result),
	.s   (fwdcontrols[1]),
	.y   (fwd_rd1));
  
  mux2 #(32) rd2fwdmux(
    .d0  (id_writedata),
	.d1  (result),
	.s   (fwdcontrols[0]),
	.y   (fwd_rd2));

  mux2 #(5) wrmux(
    .d0  (ex_rt),
    .d1  (ex_instr[15:11]),
    .s   (ex_regdst),
    .y   (srcreg));
	
  mux2 #(5) wrlmux( // writelinkmux
    .d0  (srcreg),
	.d1  (5'b11111),
	.s	 (ex_regdstra),
	.y   (ex_writereg));

  mux2 #(32) resmux(
    .d0  (wb_aluout),
    .d1  (wb_readdata),
    .s   (wb_memtoreg),
    .y   (resultd));
	
  mux2 #(32) reslmux( //registerlinkmux
    .d0  (resultd),
	.d1  (wb_pcplus4),
	.s   (wb_pctoreg),
	.y   (result));

  sign_zero_ext sze(
    .a       (id_instr[15:0]),
    .signext (id_signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (ex_signimm[31:0]),
    .shiftl16  (ex_shiftl16),
    .y         (shiftedimm[31:0]));

  // forwarding logic
	
  mux3 #(32) wrfwdmux(
    .d0 (ex_rd2),
	.d1 (mem_aluout),
	.d2 (result),
	.s  (fwdcontrols[3:2]),
	.y  (ex_writedata));
	
  mux3 #(32) srcafwdmux(
    .d0 (ex_rd1),
	.d1 (mem_aluout),
	.d2 (result),
	.s  (fwdcontrols[5:4]),
	.y  (fwd_srca));
	
  mux2 #(32) srcbmux(
    .d0 (srcb),
    .d1 (shiftedimm[31:0]),
    .s  (ex_alusrc),
    .y  (fwd_srcb));	

  mux3 #(32) srcbfwdmux(
    .d0 (ex_rd2),
	.d1 (mem_aluout),
	.d2 (result),
	.s  (fwdcontrols[3:2]),
	.y  (srcb));
	
  // ALU logic
  alu alu(
    .a       (fwd_srca),
    .b       (fwd_srcb),
    .alucont (alucontrol),
    .result  (ex_aluout),
    .zero    (ex_zero));
    
endmodule
  // ###### Jiyou Lee: End 6 ####### 
