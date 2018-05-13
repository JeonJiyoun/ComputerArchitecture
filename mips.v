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

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump;
  wire        Jal,JR;
  wire [4:0]  alucontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
		.funct      (instr[5:0]), 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.Jal        (Jal),
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
	 .Jal        (Jal),
	 .JR         (alucontrol[3]),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,Jal,
                  output [4:0] alucontrol);

  wire [2:0] aluop;
  wire       branch;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	 .Jal      (Jal),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

  assign pcsrc = branch & (op==6'b000100 ? zero : ~zero);

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,Jal,
               output [2:0] aluop);

  reg [12:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop,Jal} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0011000000110; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100000; // LW
      6'b101011: controls <= #`mydelay 13'b1000101000000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000010; // BEQ
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0010100000100; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000010000; // J
		6'b000011: controls <= #`mydelay 13'b0010000010001; // JAL
      6'b000101: controls <= #`mydelay 13'b1000010000010; // BNE
//added
      6'b001010: controls <= #`mydelay 13'b1010100001110; //slti
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [2:0] aluop,
              output reg [4:0] alucontrol);

  always @(*)
    case(aluop)
      3'b00: alucontrol <= #`mydelay 5'b0010;  // add
      3'b01: alucontrol <= #`mydelay 5'b0110;  // sub
      3'b10: alucontrol <= #`mydelay 5'b0001;  // or
		3'b111: alucontrol <=#`mydelay 5'b00111; //slti
      3'b011: case(funct)          // RTYPE
          6'b100000, 6'b000000,
          6'b100001: alucontrol <= #`mydelay 5'b0010; // ADD, NOP, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 5'b0110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 5'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 5'b0001; // OR
          6'b101010: alucontrol <= #`mydelay 5'b0111; // SLT
			 6'b101011: alucontrol <= #`mydelay 5'b10111; //SLTU
			 6'b001000: alucontrol <= #`mydelay 5'b1011; //JR
          default:   alucontrol <= #`mydelay 5'b00000; 
        endcase
		default: alucontrol <=#`mydelay 5'b00000;
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
					 input         Jal,JR,
                input  [4:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writereg, resra;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire [31:0] topc, resjal;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (topc),
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

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s    (jump),
    .y    (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (resra),
    .wd      (resjal),
    .rd1     (srca),
    .rd2     (writedata));
 mux2 #(32) jal(     //mux for JAL
    .d0      (result),
	 .d1      (pcplus4),
	 .s       (Jal),
	 .y       (resjal));
 mux2 #(32) Jr(   //mux for JR
	 .d0      (pcnext),
	 .d1      (srca),
	 .s       (JR),
	 .y       (topc)); 
 mux2 #(5) RA(
    .d0      (writereg),
	 .d1      (5'b11111),
	 .s       (Jal),
	 .y       (resra));
	 
  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (regdst),
    .y   (writereg));

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
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
