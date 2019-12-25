`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

  // ###### Jiyou Lee: Start-End: 1 ~ 12 #######
  // ######
  // cf) Only code that has been added/modified compared to my 5th Milestone code are marked with Start-End notation.
  // ######  

// pipelined MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire [5:0] op, funct;
  wire       zero;
  wire       signext;
  wire       shiftl16, regdst, regdstra, alusrc, readtaken, pcsrc, jump, link, branchtaken;
  wire [3:0] alucontrol;
  wire       regwrite, memtoreg, pctoreg;
  
  wire [5:0] fwdcontrols;
  wire [4:0] mem_writereg, wb_writereg;
  wire [4:0] ex_rs, ex_rt, id_rs, id_rt;
  wire       regwritemem;
  
  wire [4:0] ex_writereg;
  wire [1:0] flush;
  wire       stall;

  // Instantiate Forwarding Unit
  forwardingunit fdu(
    .mem_writereg  (mem_writereg),
    .wb_writereg   (wb_writereg),
    .ex_rs         (ex_rs),
    .ex_rt         (ex_rt),
    .id_rs         (id_rs),
    .id_rt         (id_rt),
    .regwritemem   (regwritemem),
    .regwrite      (regwrite),
    .fwdcontrols   (fwdcontrols));
	
  // Instantiate Hazard Detection Unit
  hazarddetectionunit hdu(
    .readtaken     (readtaken),   // from ctrl
	.branchtaken   (branchtaken), // from ctrl
    .ex_writereg   (ex_writereg),
	.id_rs         (id_rs),
	.id_rt         (id_rt),
    .stall         (stall),       // to ctrl, dp
	.flush         (flush));      // to ctrl, dp
  
  // Instantiate Controller
  controller c(
    .clk           (clk),
	.reset         (reset),
	.stall         (stall),       // from hdu
	.flush         (flush),       // from hdu
    .op            (op),          // from dp
	.funct         (funct),       // from dp
	.zero          (zero),        // from dp
	.signext       (signext),
	.shiftl16      (shiftl16),
	.regdst        (regdst),
	.regdstra      (regdstra),
	.alusrc        (alusrc),
	.readtaken     (readtaken),   // to hdu
	.pcsrc         (pcsrc),
	.jump          (jump),
	.link          (link),
	.branchtaken   (branchtaken), // to hdu
	.alucontrol    (alucontrol),
	.regwritemem   (regwritemem), // to fdu
	.memwrite      (memwrite),
	.regwrite      (regwrite),    // to fdu, dp
	.memtoreg      (memtoreg),
	.pctoreg       (pctoreg));

  // Instantiate Datapath
  datapath dp(
    .clk           (clk),
    .reset         (reset),
	.stall         (stall),       // flip-flop
	.flush         (flush),       // flip-flop
	.op            (op),          // ctrl
	.funct         (funct),       // ctrl
	.zero          (zero),        // ctrl
	.signext       (signext),
	.shiftl16      (shiftl16),
	.regdst        (regdst),
	.regdstra      (regdstra),
	.alusrc        (alusrc),
	.jump          (jump),
	.pcsrc         (pcsrc),
	.link          (link),
	.alucontrol    (alucontrol),
	.memwrite      (memwrite),
	.regwrite      (regwrite),
	.memtoreg      (memtoreg),
	.pctoreg       (pctoreg),
	.fwdcontrols   (fwdcontrols),
	.ex_writereg   (ex_writereg),
	.mem_writereg  (mem_writereg),
    .wb_writereg   (wb_writereg),
    .ex_rs         (ex_rs),
    .ex_rt         (ex_rt),
    .id_rs         (id_rs),
    .id_rt         (id_rt),
    .pc            (pc),
    .instr         (instr),
    .aluout        (memaddr),
    .writedata     (memwritedata),
    .readdata      (memreaddata));
  
endmodule



module forwardingunit(input      [4:0] mem_writereg, wb_writereg,
					  input      [4:0] ex_rs, ex_rt, id_rs, id_rt,
					  input            regwritemem, regwrite,
					  output reg [5:0] fwdcontrols); 
  
  wire [1:0] fducontrols = {regwritemem, regwrite};

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


module hazarddetectionunit(
						   input        readtaken,
						   input        branchtaken,
						   input  [4:0] ex_writereg,
                           input  [4:0] id_rs, id_rt,
                           output       stall,
						   output [1:0] flush);
	
	assign stall = (readtaken) && ((ex_writereg == id_rs) || (ex_writereg == id_rt));
	assign flush[0] = stall || branchtaken;
	assign flush[1] = branchtaken;
	
endmodule



module controller(input        clk, reset, stall,
				  input  [1:0] flush,
				  input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16, regdst, regdstra, alusrc, readtaken, pcsrc, jump, link, branchtaken,
				  output [3:0] alucontrol,
                  output       regwritemem, memwrite,
                  output       regwrite, memtoreg, pctoreg);
				  
  // ###### Jiyou Lee: Start 1 #######
  wire [26:0] idexctrl_flush;
  wire [15:0] id_controls;
  // ###### Jiyou Lee: End 1 #######
  wire  [5:0] id_op, id_funct;

  // ###### Jiyou Lee: Start 2 #######
  wire [14:0] ex_controls;
  wire  [2:0] ex_aluop;
  // ###### Jiyou Lee: End 2 #######
  wire  [5:0] ex_op, ex_funct;
  wire        ex_branch, ex_le;
  
  wire  [3:0] mem_controls;
  
  wire  [2:0] wb_controls;
  
  // controls: {id_signext, shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, memread, memwrite, memtoreg, pctoreg, jump, aluop}
  assign id_op = op;
  assign id_funct = funct;
  // ###### Jiyou Lee: Start 3 #######
  assign signext = id_controls[15];
  // ###### Jiyou Lee: End 3 #######
  
  // controls: {shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, readtaken, memwrite, memtoreg, pctoreg, jump, aluop}
  // ###### Jiyou Lee: Start 4 #######
  assign shiftl16 = ex_controls[14];
  assign {regdst, regdstra, alusrc, ex_branch, ex_le, readtaken} = ex_controls[12:7];
  assign {jump, ex_aluop} = ex_controls[3:0];
  // ###### Jiyou Lee: End 4 #######
  
  // controls: {regwrite, memwrite, memtoreg, pctoreg} 
  assign {regwritemem, memwrite} = mem_controls[3:2];
  
  // controls: {regwrite, memtoreg, pctoreg}
  assign {regwrite, memtoreg, pctoreg} = wb_controls;
  
  assign branchtaken = pcsrc || jump || link;

  maindec md(
    .op       (op),
    .controls (id_controls));
	
  aludec ad( 
    .funct      (ex_funct),
    .aluop      (ex_aluop), 
    .alucontrol (alucontrol));

  pcdec  pd(
	.op     (ex_op),
	.funct  (ex_funct),
	.branch (ex_branch),
	.zero   (zero),
	.le     (ex_le),
	.pcsrc  (pcsrc),
	.link   (link));

  // pipelining logic
  // ###### Jiyou Lee: Start 5 #######
  mux2 #(27) IDEXCTRLmux(
    .d0    ({id_op, id_funct, id_controls[14:0]}),
	.d1    (27'b0),
  // ###### Jiyou Lee: End 5 #######
	.s     (flush[0]),
	.y     (idexctrl_flush));

  // ###### Jiyou Lee: Start 6 #######
  flopr #(27) IDEXCTRL(
  // ###### Jiyou Lee: End 6 #######
    .clk   (clk),
	.reset (reset),
    .d     (idexctrl_flush),
	.q     ({ex_op, ex_funct, ex_controls}));
	
  flopr  #(4) EXMEMCTRL(
    .clk   (clk),
	.reset (reset),
  // ###### Jiyou Lee: Start 7 #######
	.d     ({ex_controls[13], ex_controls[6:4]}),
  // ###### Jiyou Lee: End 7 #######
	.q     (mem_controls));
	
  flopr #(3) MEMWBCTRL(
    .clk   (clk),
	.reset (reset),
	.d     ({mem_controls[3], mem_controls[1:0]}),
	.q     (wb_controls));

endmodule



module maindec(input       [5:0]  op,
  // ###### Jiyou Lee: Start 8 #######
               output reg [15:0] controls);
  // ###### Jiyou Lee: End 8 #######

//  assign {signext, shiftl16, regwrite, regdst, regdstra, alusrc, branch, le, readtaken, memwrite,
//          memtoreg, pctoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 16'b0011000100000011; // Rtype
      6'b100011: controls <= #`mydelay 16'b1010010010100000; // LW
      6'b101011: controls <= #`mydelay 16'b1000010001000000; // SW
	  6'b000011: controls <= #`mydelay 16'b0010100000011000; // JAL
      6'b000100,
	  6'b000101: controls <= #`mydelay 16'b1000001000000001; // BEQ, BNE: only difference is pcsrc
      6'b001000, 
      6'b001001: controls <= #`mydelay 16'b1010010000000000; // ADDI, ADDIU: only difference is exception
  // ###### Jiyou Lee: Start 9 #######
      6'b001010: controls <= #`mydelay 16'b1010010000000100; // SLTI
  // ###### Jiyou Lee: End 9 #######
	  6'b001101: controls <= #`mydelay 16'b0010010000000010; // ORI
      6'b001111: controls <= #`mydelay 16'b0110010000000000; // LUI
      6'b000010: controls <= #`mydelay 16'b0000000000001000; // J
      default:   controls <= #`mydelay 16'bxxxxxxxxxxxxxxxx; // ???
    endcase

endmodule



module aludec(input      [5:0] funct,
  // ###### Jiyou Lee: Start 10 #######
              input      [2:0] aluop,
  // ###### Jiyou Lee: End 10 #######
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
  // ###### Jiyou Lee: Start 11 #######
      3'b000: alucontrol <= #`mydelay 4'b0100;  // add
      3'b001: alucontrol <= #`mydelay 4'b1100;  // sub
      3'b010: alucontrol <= #`mydelay 4'b0010;  // or
      3'b011: case(funct)          // RTYPE
  // ###### Jiyou Lee: End 11 #######
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
  // ###### Jiyou Lee: Start 12 #######
	  3'b100: alucontrol <= #`mydelay 4'b1110;        // SLTI
	  default: alucontrol <= #`mydelay 4'bxxxx;       // ???
  // ###### Jiyou Lee: End 12 #######
    endcase
    
endmodule



module pcdec(input  [5:0] op, funct,
			 input        branch, zero,
			 input        le,
			 output       pcsrc, link);
			 
  assign pcsrc = (branch && zero && ~op[0]) || (branch && ~zero && op[0]);
  assign link = le & (funct == 6'b001000);

endmodule
  


module datapath(input         clk, reset, stall,
			    input   [1:0] flush,
				output  [5:0] op, funct,                              // ctrl
				output        zero,                                   // ctrl
				input         signext,
				input         shiftl16, regdst, regdstra, alusrc, jump, pcsrc, link,
				input   [3:0] alucontrol,
				input         memwrite,
				input         regwrite, memtoreg, pctoreg,
				input   [5:0] fwdcontrols,
				output  [4:0] ex_writereg, mem_writereg, wb_writereg, // fdu
				output  [4:0] ex_rs, ex_rt, id_rs, id_rt,             // fdu
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire  [4:0]  srcreg, writereg;
  wire  [31:0] pcnext, pcnextj, pcnextbr, pcplus4, pcbranch;
  wire  [31:0] signimm, shiftedimm;
  wire  [31:0] srcb;
  wire  [31:0] resultd, result;
  wire         shift;
 
  // pipelining declarations
  wire  [63:0] ifid_d, ifid_q, ifid_flush;
  wire  [31:0] id_pcplus4, id_instr;
  wire  [31:0] id_rd1, id_rd2;
  
  wire [153:0] idex_d, idex_q, idex_flush;
  wire  [31:0] ex_pcplus4, ex_rd1, ex_rd2, ex_signimm;
  wire  [31:0] ex_signimmsh, ex_pcbranch, ex_pcnextj;
  wire  [31:0] ex_aluout, ex_writedata;
  wire  [25:0] ex_instr;
  
  wire [100:0] exmem_d, exmem_q;
  wire  [31:0] mem_aluout, mem_pcplus4;
  
  wire [100:0] memwb_d, memwb_q;
  wire  [31:0] wb_aluout, wb_readdata, wb_pcplus4;
  wire         wb_memtoreg, wb_pctoreg;
  
  // forwarding declarations
  wire  [31:0] fwd_srca, fwd_srcb;
  wire  [31:0] fwd_rd1, fwd_rd2;
  
  // pipelining assignments
  assign ifid_d = {pcplus4, instr};
  assign {id_pcplus4, id_instr} = ifid_q;
  assign op = id_instr[31:26];
  assign funct = id_instr[5:0];
    
  assign idex_d = {id_pcplus4, fwd_rd1, fwd_rd2, signimm, id_instr[25:0]};
  assign {ex_pcplus4, ex_rd1, ex_rd2, ex_signimm, ex_instr} = idex_q;
  assign ex_pcnextj = {ex_pcplus4[31:28], ex_instr, 2'b00};

  assign exmem_d = {ex_aluout, ex_writedata, ex_writereg, ex_pcplus4};
  assign {mem_aluout, writedata, mem_writereg, mem_pcplus4} = exmem_q;
  assign aluout = mem_aluout;
  
  assign memwb_d = {mem_writereg, mem_aluout, readdata, mem_pcplus4};
  assign {wb_writereg, wb_aluout, wb_readdata, wb_pcplus4} = memwb_q;

  // forwarding assignments
  assign id_rs = id_instr[25:21];
  assign id_rt = id_instr[20:16];
  
  assign ex_rs = ex_instr[25:21];
  assign ex_rt = ex_instr[20:16];

  // pipelining logic
  mux2 #(64) IFIDmux(
    .d0    (ifid_d),
	.d1    (64'b0),
	.s     (flush[1]),
	.y     (ifid_flush));
  
  flopenr #(64) IFID(
	.clk   (clk),
	.reset (reset),
	.en    (~stall),
	.d     (ifid_flush),
	.q     (ifid_q));
	
  mux2 #(154) IDEXmux(
    .d0    (idex_d),
	.d1    (154'b0),
	.s     (flush[0]),
	.y     (idex_flush));
	
  flopr #(154) IDEX(
	.clk   (clk),
	.reset (reset),
	.d     (idex_flush),
	.q     (idex_q));
	
  flopr #(101) EXMEM(
	.clk   (clk),
	.reset (reset),
	.d     (exmem_d),
	.q     (exmem_q));
	
  flopr #(101) MEMWB(
	.clk   (clk),
	.reset (reset),
	.d     (memwb_d),
	.q     (memwb_q));
  
  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	.en    (~stall),
    .d     (pcnext),
    .q     (pc)); 

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (ex_signimm),
    .y (ex_signimmsh));
	
  adder pcadd2(
    .a (ex_pcplus4),
    .b (ex_signimmsh),
    .y (ex_pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (ex_pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcjmux(
    .d0   (pcnextbr),
    .d1   (ex_pcnextj),
    .s    (jump),
    .y    (pcnextj));
	
  mux2 #(32) pcmux(
    .d0	  (pcnextj),
	.d1	  (ex_aluout),
	.s	  (link),
	.y	  (pcnext));
	
  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (id_rs),
    .ra2     (id_rt),
    .wa      (wb_writereg),
    .wd      (result),
    .rd1     (id_rd1),
    .rd2     (id_rd2));
	
  mux2 #(32) rd1fwdmux(
    .d0  (id_rd1),
	.d1  (result),
	.s   (fwdcontrols[1]),
	.y   (fwd_rd1));
  
  mux2 #(32) rd2fwdmux(
    .d0  (id_rd2),
	.d1  (result),
	.s   (fwdcontrols[0]),
	.y   (fwd_rd2));

  mux2 #(5) wrmux(
    .d0  (ex_rt),
    .d1  (ex_instr[15:11]),
    .s   (regdst),
    .y   (srcreg));
	
  mux2 #(5) wrlmux( // writelinkmux
    .d0  (srcreg),
	.d1  (5'b11111),
	.s	 (regdstra),
	.y   (ex_writereg));

  mux2 #(32) resmux(
    .d0  (wb_aluout),
    .d1  (wb_readdata),
    .s   (memtoreg),
    .y   (resultd));
	
  mux2 #(32) reslmux( //registerlinkmux
    .d0  (resultd),
	.d1  (wb_pcplus4),
	.s   (pctoreg),
	.y   (result));

  sign_zero_ext sze(
    .a       (id_instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (ex_signimm[31:0]),
    .shiftl16  (shiftl16),
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
    .s  (alusrc),
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
    .zero    (zero));
    
endmodule
