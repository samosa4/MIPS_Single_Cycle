// Hardware Realisation of a Computer System - 6037CEM
// Mips Single Cyvle
// Esosa M. Johnson Ikponmwosa

`timescale 1ns/1ps

module mips(input  logic        clk, reset,
            output logic [31:0] pc,
            input  logic [31:0] instr,
            output logic        memwrite,
            output logic [31:0] aluout, writedata,
            input  logic [31:0] readdata);

  // Control signals
  logic       memtoreg, alusrc, regdst, zeroext,
              regwrite, jump, pcsrc, zero, gtz, ltz;
  logic [4:0] alucontrol;

  // Instantiate controller and datapath
  controller c(instr[31:26], instr[5:0], zero,
               gtz, ltz, memtoreg, memwrite, pcsrc,
               alusrc, regdst, regwrite, jump,
               zeroext, alucontrol);
  datapath dp(clk, reset, zeroext, memtoreg, pcsrc,
              alusrc, regdst, regwrite, jump,
              alucontrol, instr[10:6],
              zero, gtz, ltz, pc, instr,
              aluout, writedata, readdata);
endmodule

// Control Unit
module controller(input  logic [5:0] op, funct,
                  input  logic       zero, gtz, ltz,
                  output logic       memtoreg, memwrite,
                  output logic       pcsrc, alusrc,
                  output logic       regdst, regwrite,
                  output logic       jump, zeroext,
                  output logic [4:0] alucontrol);

  logic [3:0] aluop;
  logic       branch, bgtz, bltz;

  // Main Decoder and ALU Decoder
  maindec md(op, memtoreg, memwrite, branch,
             alusrc, zeroext, regdst, regwrite, jump, bgtz, bltz, aluop);
  aludec  ad(funct, aluop, alucontrol);

  // Determine PC Source
  assign pcsrc = (branch & zero) | (bgtz & gtz) | (bltz & ltz);
endmodule

// Main Decoder
module maindec(input  logic [5:0] op,
               output logic       memtoreg, memwrite,
               output logic       branch, alusrc, zeroext,
               output logic       regdst, regwrite,
               output logic       jump, bgtz, bltz,
               output logic [3:0] aluop);

  logic [13:0] controls;

  assign {regwrite, regdst, zeroext, alusrc, branch, memwrite,
          memtoreg, jump, aluop, bgtz, bltz} = controls;

  always_comb
    case(op)
      6'b000000: controls <= 14'b11000000100000; // RTYPE
      6'b100011: controls <= 14'b10010010000000; // LW
      6'b101011: controls <= 14'b00010100000000; // SW
      6'b000100: controls <= 14'b00001000000100; // BEQ
      6'b001000: controls <= 14'b10010000000000; // ADDI
      6'b001101: controls <= 14'b10110000001000; // ORI
      6'b001110: controls <= 14'b10110000001100; // XORI
      6'b000010: controls <= 14'b00000001000000; // J
      6'b001100: controls <= 14'b10110000010000; // ANDI
      6'b000111: controls <= 14'b00000000000110; // BGTZ
      6'b000001: controls <= 14'b00000000000101; // BLTZ
      default:   controls <= 14'bxxxxxxxxxxxxxx; // illegal op
    endcase
endmodule

// ALU Decoder
module aludec(input  logic [5:0] funct,
              input  logic [3:0] aluop,
              output logic [4:0] alucontrol);
  
  // Decode based on ALUOp and funct
  always_comb
    case(aluop)
      4'b0000: alucontrol <= 5'b00010;  // add (for lw/sw/addi)
      4'b0001: alucontrol <= 5'b10010;  // sub (for beq/bgtz/bltz)
      4'b0010: alucontrol <= 5'b00001;  // or  (for ori)
      4'b0011: alucontrol <= 5'b00100;  // xor (for xori)
      4'b0100: alucontrol <= 5'b00000;  // and (for andi)
      default: case(funct)           // R-type instructions
          6'b100000: alucontrol <= 5'b00010; // add
          6'b100010: alucontrol <= 5'b10010; // sub
          6'b100100: alucontrol <= 5'b00000; // and
          6'b100101: alucontrol <= 5'b00001; // or
          6'b101010: alucontrol <= 5'b10011; // slt
          6'b000010: alucontrol <= 5'b00101; // srl
          6'b000110: alucontrol <= 5'b00110; // srlv
          6'b001000: alucontrol <= 5'b01000; // jr
          default:   alucontrol <= 5'bxxxxx; // ???
        endcase
    endcase
endmodule

// Datapath Module
module datapath(input  logic        clk, reset, zeroext,
                input  logic        memtoreg, pcsrc,
                input  logic        alusrc, regdst,
                input  logic        regwrite, jump,
                input  logic [4:0]  alucontrol,
                input  logic [4:0]  shamt,
                output logic        zero, gtz, ltz,
                output logic [31:0] pc,
                input  logic [31:0] instr,
                output logic [31:0] aluout, writedata,
                input  logic [31:0] readdata);

  // Intermediate signals
  logic [4:0]  writereg;
  logic [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  logic [31:0] signimm, zeroimm, signimmsh, imm;
  logic [31:0] srca, srcb;
  logic [31:0] result;
  logic        jr;

  // Next PC logic
  flopr #(32) pcreg(clk, reset, pcnext, pc);
  adder       pcadd1(pc, 32'b100, pcplus4);
  sl2         immsh(imm, signimmsh);
  adder       pcadd2(pcplus4, signimmsh, pcbranch);
  mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc, pcnextbr);
  mux4 #(32)  pcmux(pcnextbr, srca,{pcplus4[31:28], 
                    instr[25:0], 2'b00}, srca, {jump, jr}, pcnext);

  // Register file logic
  regfile     rf(clk, regwrite, instr[25:21], instr[20:16], 
                 writereg, result, srca, writedata);
  mux2 #(5)   wrmux(instr[20:16], instr[15:11],
                    regdst, writereg);
  mux2 #(32)  resmux(aluout, readdata, memtoreg, result);
  signext     se(instr[15:0], signimm);
  zeroext     ze(instr[15:0], zeroimm);
  mux2 #(32)  signorzero(signimm, zeroimm, zeroext, imm);

  // ALU logic
  mux2 #(32)  srcbmux(writedata, imm, alusrc, srcb);
  alu         alu(srca, srcb, shamt, alucontrol, aluout, zero, gtz, ltz, jr);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [4:0]  ra1, ra2, wa3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clk
  // register 0 hardwired to 0
  // note: for pipelined processor, write third port
  // on falling edge of clk

  always_ff @(posedge clk)
    if (we3) rf[wa3] <= wd3;	

  assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule

// Adder
module adder(input  logic [31:0] a, b,
             output logic [31:0] y);

  assign y = a + b;
endmodule

// shift left by 2
module sl2(input  logic [31:0] a,
           output logic [31:0] y);

  assign y = {a[29:0], 2'b00};
endmodule

// Sign Extension
module signext(input  logic [15:0] a,
               output logic [31:0] y);
              
  assign y = {{16{a[15]}}, a};
endmodule

// Zero Extension
module zeroext(input  logic [15:0] a,
                 output logic [31:0] y);

  assign y = {16'b0, a};
endmodule

// D flip-flop with synchronous reset
module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

// 2-to-1 multiplexer
module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

// 4-to-1 multiplexer
module mux4 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2, d3,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0);
endmodule

// ALU Module
module alu(input  logic [31:0] a, b,
           input  logic [4:0]  shamt,
           input  logic [4:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero, gtz, ltz, jr);

  logic [31:0] condinvb, sum;

  // Conditional inversion of B based on ALU control signal
  assign condinvb = alucontrol[4] ? ~b : b;
  
  // Sum calculation
  assign sum = a + condinvb + alucontrol[4];  

  always_comb
    case (alucontrol[3:0])
      4'b0000: result = a & b;        // AND
      4'b0001: result = a | b;        // OR
      4'b0010: result = sum;          // ADD/SUB
      4'b0011: result = sum[31];      // SLT
      4'b0100: result = a ^ b;        // XOR
      4'b0101: result = b >> shamt;   // SRL
      4'b0110: result = b >> a[4:0];  // SRLV
    endcase
    
    // JR control signal
    always_comb
      case (alucontrol[4:0])
        5'b01000: jr <= 1;            // JR
        default:  jr <= 0;
      endcase

  // Zero, Greater than zero, Less than zero flags
  assign zero = (result == 32'b0);
  assign gtz = (~a[31]) & (~zero); // bgtz
  assign ltz = (a[31]) & (~zero); // bltz
endmodule