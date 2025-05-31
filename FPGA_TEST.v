//Top-level module for multi-cycle microprocessor
module FPGA_TEST( 
	 input [17:0] SW,          
    input [3:0] KEY,         
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    output [17:0] LEDR,      
    output [7:0] LEDG        
);
    wire [31:0] PC_out, ALU_out;
    wire reset;              

 
    Datapath_Multi_cycle_Processor dut (.clk(KEY[0]),.in_reset(SW[0]),.PC_out(PC_out),.ALU_out(ALU_out)    
    );

    hex_ssd H0 (.BIN(ALU_out[3:0]),   .SSD(HEX0));
    hex_ssd H1 (.BIN(ALU_out[7:4]),   .SSD(HEX1));
    hex_ssd H2 (.BIN(ALU_out[11:8]),  .SSD(HEX2));
    hex_ssd H3 (.BIN(ALU_out[15:12]), .SSD(HEX3));
    hex_ssd H4 (.BIN(ALU_out[19:16]), .SSD(HEX4));
    hex_ssd H5 (.BIN(ALU_out[23:20]), .SSD(HEX5));
    hex_ssd H6 (.BIN(PC_out[3:0]), .SSD(HEX6));
    hex_ssd H7 (.BIN(PC_out[7:4]), .SSD(HEX7));


    assign LEDG[0] = ALU_out[0];          
    assign LEDR[15:0] = PC_out[15:0];    
    assign LEDR[17:16] = 2'b00;           
    assign LEDG[7:1] = 7'b0000000;    

endmodule

// Seven-segment display decoder
module hex_ssd (
    input [3:0] BIN,
    output reg [0:6] SSD
);
    always @(*) begin
        case (BIN)
            4'h0: SSD = 7'b0000001;
            4'h1: SSD = 7'b1001111;
            4'h2: SSD = 7'b0010010;
            4'h3: SSD = 7'b0000110;
            4'h4: SSD = 7'b1001100;
            4'h5: SSD = 7'b0100100;
            4'h6: SSD = 7'b0100000;
            4'h7: SSD = 7'b0001111;
            4'h8: SSD = 7'b0000000;
            4'h9: SSD = 7'b0001100;
            4'hA: SSD = 7'b0001000;
            4'hB: SSD = 7'b1100000;
            4'hC: SSD = 7'b0110001;
            4'hD: SSD = 7'b1000010;
            4'hE: SSD = 7'b0110000;
            4'hF: SSD = 7'b0111000;
            default: SSD = 7'b1111111;
        endcase
    end
endmodule



// Multi-cycle microprocessor
module  Datapath_Multi_cycle_Processor(clk,in_reset,PC_out,ALU_out);
    input clk, in_reset;
    output [31:0] PC_out,ALU_out;
    wire [31:0] ALU_in_B,ALU_in_A,B_data,mux_2_out,Jump_addr;
    wire [31:0] PC_in,Mem_Read_data,instruction,MDR_out,ALU_out_hold;
    wire PCWr,IRwrite,MemRead;
    wire [27:0] jump_28_bit;
    
    wire [2:0]ALUop;
    wire [31:0] mux_1_out,W_RD1, W_RD2,Extend_out,Branch_addr,A_data;
    wire [4:0] mux_3_out;
    wire PCWrite,zero,PCWrcond,and_out,reset;
    
    wire Iord,MemWrite,MemtoReg,RegWrite,RegDst,ALUSrcA;
    wire [1:0] ALUSrcB,PCSource;
    wire [2:0] Operation_ALU;
    
    Program_Counter     comp1(clk, reset,PCWr,PC_in, PC_out);
    Mux_32_bit          comp2(PC_out, ALU_out_hold, mux_1_out, Iord);
    Data_Memory         comp3(clk,mux_1_out, B_data, Mem_Read_data, MemRead, MemWrite);
    holding_reg         comp4(instruction, Mem_Read_data, IRwrite, clk, reset);
    holding_reg         comp5(MDR_out, Mem_Read_data, 1'b1, clk, reset);
    Mux_32_bit          comp6(MDR_out,ALU_out_hold, mux_2_out, MemtoReg);

    Register_File       comp7(clk,instruction[25:21], instruction[20:16], mux_3_out, W_RD1, W_RD2, mux_2_out, RegWrite);
    Mux_5_bit           comp8(instruction[20:16], instruction[15:11], mux_3_out, RegDst);
    Sign_Extension      comp9(instruction[15:0], Extend_out);
    shift_left_2        comp10(Extend_out, Branch_addr);
    holding_reg         comp11(A_data, W_RD1, 1'b1, clk, reset);
    holding_reg         comp12(B_data, W_RD2, 1'b1, clk, reset);
    Mux_32_bit          comp13(PC_out, A_data, ALU_in_A, ALUSrcA);
    Mux4_32_bit         comp14(B_data, 32'd4,Extend_out,Branch_addr , ALU_in_B, ALUSrcB);
    alu                 comp15(Operation_ALU, ALU_in_A, ALU_in_B, ALU_out,zero);
    holding_reg         comp16(ALU_out_hold, ALU_out , 1'b1, clk, reset);
    
    shift_left_2_28bit  comp17(instruction[25:0], jump_28_bit);
    
    concate             comp18(PC_out[31:28],jump_28_bit,Jump_addr);
    Mux4_32_bit         comp19(ALU_out, ALU_out_hold,Jump_addr, 32'b0, PC_in, PCSource);
    
    controller          comp20(in_reset,instruction[31:26], reset,clk,PCWrite,Iord,MemRead,MemWrite,IRwrite,MemtoReg,RegWrite,RegDst,ALUSrcA,ALUSrcB,PCSource,ALUop,PCWrcond);
    ALU_Control         comp21(ALUop,instruction[5:0],Operation_ALU);
    and                 comp22(and_out,zero,PCWrcond);
    or                  comp23(PCWr,and_out,PCWrite);
endmodule

module controller(in_reset,opcode, reset,clk,PCWrite,Iord,MemRead,MemWrite,IRwrite,MemtoReg,RegWrite,RegDst,ALUSrcA,ALUSrcB,PCSource,ALUop,PCWrcond);

  // ~~~~~~~~~~~~~~~~~~~ PORTS ~~~~~~~~~~~~~~~~~~~ //

  // opcode, clock, and reset inputs
  input [5:0] opcode;	// from instruction register
  input	clk,in_reset;

  // control signal outputs
  output reg PCWrite,Iord,MemRead,MemWrite,IRwrite,MemtoReg,RegWrite,RegDst,ALUSrcA;
  output reg [1:0] ALUSrcB,PCSource;
  output reg [2:0] ALUop;
  output reg PCWrcond;
  output reg reset;
  // ~~~~~~~~~~~~~~~~~~~ REGISTER ~~~~~~~~~~~~~~~~~~~ //

  // 4-bit state register
  reg [3:0]	state;

  // ~~~~~~~~~~~~~~~~~~~ PARAMETERS ~~~~~~~~~~~~~~~~~~~ //

  // state parameters
  parameter s0  = 4'd0;
  parameter s1  = 4'd1;
  parameter s2  = 4'd2;
  parameter s3  = 4'd3;
  parameter s4  = 4'd4;
  parameter s5  = 4'd5;
  parameter s6  = 4'd6;
  parameter s7  = 4'd7;
  parameter s8  = 4'd8;
  parameter s9  = 4'd9;
  parameter s10  = 4'd10;
  parameter s_Reset  = 4'd11;	// reset
  


  // opcode[5:4] parameters
  parameter J       = 6'b000010;	// Jump or NOP
  parameter R       = 6'b000000;	// R-type
  parameter BEQ     = 6'b000100;	// Branch
  parameter BNE     = 6'b000101;    // Branch
  parameter SW      = 6'b101011;	// I-type
  parameter LW      = 6'b100011;     // I-type
  parameter ADDI    = 6'b001000;    // I-type
  

  // OP code control for ALU

  parameter OP_R_TYPE  = 3'b000;
  parameter OP_I_TYPE  = 3'b001;
  parameter OP_J_TYPE  = 3'b010;
  parameter OP_BR_TYPE = 3'b011;
  parameter OP_IF_TYPE = 3'b100;
  parameter OP_ID_TYPE = 3'b101;
  parameter OP_RS_TYPE = 3'b110;


  // ~~~~~~~~~~~~~~~~~~~ STATE MACHINE ~~~~~~~~~~~~~~~~~~~ //

  // control state machine
  always @(posedge clk or posedge in_reset) 
  begin

    // check for reset signal. If set, write zero to PC and switch to Reset State on next CC.
    if (in_reset) begin
      PCWrite=0;
      Iord=0;
      MemRead=1;
      MemWrite=0;
      IRwrite=1;
      MemtoReg=0;
      RegWrite=0;
      RegDst=0;
      ALUSrcA=0;
      ALUSrcB=2'b01;
      PCSource=2'b00;
      ALUop=OP_RS_TYPE;
      PCWrcond=0;
      reset =1;
      state <= s_Reset;
    end
    else 
    begin	// if reset signal is not set, check state at pos edge
      case (state)
       s_Reset:
            begin
              PCWrite=0;
              Iord=0;
              MemRead=1;
              MemWrite=0;
              IRwrite=1;
              MemtoReg=0;
              RegWrite=0;
              RegDst=0;
              ALUSrcA=0;
              ALUSrcB=2'b01;
              PCSource=2'b00;
              ALUop=OP_RS_TYPE;
              PCWrcond=0;
              reset =0;
              state <= s0;
              $display("state Reset");
            end
      s0:
         begin 
            Iord=0;
            MemRead=1;
            MemWrite=0;
            IRwrite=1;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b01;
            PCSource=2'b00;
            ALUop=OP_IF_TYPE;
            PCWrcond=0;
            state <= s1;
            PCWrite=1;
            $display("state 0");
         end
      s1:
         begin 
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b11;
            PCSource=2'b00;
            ALUop=OP_ID_TYPE;
            PCWrcond=0;
            $display("state 1");
            case(opcode[5:0])
                    J:  state <= s9;
                    R:  state <= s6;
                    SW:  state <= s2;
		            LW:  state <= s2;
		            ADDI: state <= s2;
                    BEQ: state <= s8;
            endcase
         end

      s2:
         begin 
            PCWrite=0;
            Iord=1;
            MemRead=1;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b10;
            PCSource=2'b00;
            ALUop=OP_I_TYPE;
            PCWrcond=0;
            $display("state 2");
            if(opcode[5:0]== ADDI)
                begin
                  state <= s10;
                  $display("ADDI state");
                end
            else if(opcode[5:0]== SW)
                     begin
                        state <= s5;
                        $display("SW state");
                     end
                 else 
                     begin
                        state <= s3;
                        $display("SW state");
                     end
            $display("state 2");
         end
      s3:
         begin 
            PCWrite=0;
            Iord=1;
            MemRead=1;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b10;
            PCSource=2'b00;
            ALUop=OP_I_TYPE;
            PCWrcond=0;
            state <= s4;
            $display("state 3");
         end
      s4:
         begin 
            PCWrite=0;
            Iord=1;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=1;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b10;
            PCSource=2'b00;
            ALUop=OP_I_TYPE;
            PCWrcond=0;
            state <= s0;
            $display("state 4");
         end
      s5:
         begin 
            PCWrite=0;
            Iord=1;
            MemRead=0;
            MemWrite=1;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b10;
            PCSource=2'b00;
            ALUop=OP_I_TYPE;
            PCWrcond=0;
            state <= s0;
            $display("state 5");
         end

      s6:
         begin 
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b00;
            PCSource=2'b00;
            ALUop=OP_R_TYPE;
            PCWrcond=0;
            state <= s7;
            $display("state 6");
         end
 
      s7:
         begin 
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=1;
            RegWrite=1;
            RegDst=1;
            ALUSrcA=1;
            ALUSrcB=2'b00;
            PCSource=2'b00;
            ALUop=OP_R_TYPE;
            PCWrcond=0;
            state <= s0;
            $display("state 7");
         end

      s8:
         begin 
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b00;
            PCSource=2'b01;
            ALUop=OP_BR_TYPE;
            PCWrcond=1;
            state <= s0;
            $display("state 8");
         end

      s9:
         begin 
            PCWrite=1;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b00;
            PCSource=2'b10;
            ALUop=OP_J_TYPE;
            PCWrcond=0;
            state <= s0;
            $display("state 9");
         end
      s10:
         begin 
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=1;
            RegWrite=1;
            RegDst=0;
            ALUSrcA=1;
            ALUSrcB=2'b00;
            PCSource=2'b00;
            ALUop=OP_R_TYPE;
            PCWrcond=0;
            state <= s0;
            $display("state 7");
         end
        default: begin
            PCWrite=0;
            Iord=0;
            MemRead=0;
            MemWrite=0;
            IRwrite=0;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b01;
            PCSource=2'b00;
            ALUop=OP_RS_TYPE;
            PCWrcond=0;
            $display("state default control");
          state <= s_Reset;
        end
      endcase
    end
  end
endmodule

module ALU_Control(Op_intstruct,ints_function,ALUOp);
    input [5:0] ints_function;
    input [2:0] Op_intstruct;
    output reg [2:0] ALUOp;
      // OP code control for ALU

  parameter OP_R_TYPE  = 3'b000;
  parameter OP_I_TYPE  = 3'b001;
  parameter OP_J_TYPE  = 3'b010;
  parameter OP_BR_TYPE = 3'b011;
  parameter OP_IF_TYPE = 3'b100;
  parameter OP_ID_TYPE = 3'b101;
  parameter OP_RS_TYPE = 3'b110;

    always @(*)
    begin
        case(Op_intstruct)
            OP_R_TYPE:   // R -Type Instruction look at fuction
                begin
                    ALUOp =3'b000;
                    if(ints_function==6'b100000) // add
                        begin
                            ALUOp =3'b000;
                            $display("fuction Add");
                        end
                                  
                    if(ints_function==6'b100010) // sub 
                        begin
                            ALUOp =3'b001;
                            $display("fuction sub");
                        end
                                   
                    if(ints_function==6'b100100) // and
                        begin
                            ALUOp =3'b010;
                            $display("fuction and");
                        end
                                   
                    if(ints_function==6'b100101) // or
                        begin
                            ALUOp =3'b010;
                            $display("fuction or ");
                        end
                end
            OP_I_TYPE:  
                begin
                    ALUOp =3'b000;
                    $display("LW or SW");
                end
            OP_J_TYPE: 
                begin
                    ALUOp =3'b000;
                    $display("Jump");
                end
            OP_BR_TYPE: // beq instruction
                begin
                    ALUOp =3'b111;
                    $display("BEQ or BNE");
                end
            OP_IF_TYPE: // Store Instruction
                begin
                    ALUOp =3'b000;
                    $display("Add IF");
                end
            OP_ID_TYPE: // addi Instruction
                begin
                    ALUOp =3'b000;
                    $display("add ID");
                end
            OP_RS_TYPE:  //bne
                begin
                    ALUOp =3'b111;
                    $display("Reset OP");
                end
            default :
                begin
                    ALUOp =3'b000;
		            $display("ALU default");
                end
        endcase
    end
endmodule
module Mux_5_bit (in0, in1, mux_out, select);
	parameter N = 5;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

module Program_Counter (clk, reset,PC_write ,PC_in, PC_out);
	input clk, reset,PC_write;
	input [31:0] PC_in;
	output reg [31:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else if (PC_write==1'b1)
			PC_out<=PC_in;
	end
endmodule

module alu(
	input [2:0] alufn,
	input [31:0] ra,
	input [31:0] rb_or_imm,
	output reg [31:0] aluout,
	output reg zero);
	parameter	ALU_OP_ADD	    = 3'b000,
			    ALU_OP_SUB	    = 3'b001,
			    ALU_OP_AND	    = 3'b010,
			    ALU_OP_OR	    = 3'b011,
			    ALU_OP_XOR	    = 3'b100,
			    ALU_OP_LW	    = 3'b101,
			    ALU_OP_SW	    = 3'b110,
			    ALU_OP_BEQ	    = 3'b111;

    always @(*) 
        begin
		  case(alufn)
			ALU_OP_ADD 	    : aluout = ra + rb_or_imm;
			ALU_OP_SUB 	    : aluout = ra - rb_or_imm;
			ALU_OP_AND 	    : aluout = ra & rb_or_imm;
			ALU_OP_OR	    : aluout = ra | rb_or_imm;
			ALU_OP_XOR	    : aluout = ra ^ rb_or_imm;
			ALU_OP_LW	    : aluout = ra + rb_or_imm;
			ALU_OP_SW	    : aluout = ra + rb_or_imm;
			ALU_OP_BEQ	    : begin
					            zero = (ra==rb_or_imm)?1'b1:1'b0;
					            aluout = ra - rb_or_imm;
					          end
		  endcase
        end
endmodule

module Register_File (clk,read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [31:0] write_data;
	input  clk,RegWrite;
	reg checkRegWrite;
	output reg [31:0] read_data_1, read_data_2;
	reg [31:0] Regfile [31:0];
	integer k;
	initial 
	    begin
	        for (k=0; k<32; k=k+1) 
			    begin
				    Regfile[k] = 32'd10;
			    end
			Regfile[8]=32'd1;
			Regfile[9]=32'd2;
			Regfile[10]=32'd3; //$t2
			Regfile[11]=32'd4; //$t3
			
			
			Regfile[17]=32'd99;
			Regfile[18]=32'd60;
			Regfile[19]=32'd30;
	    end
	
	//assign read_data_1 = Regfile[read_addr_1];
        always @(read_data_1 or Regfile[read_addr_1])
	        begin
	          if (read_addr_1 == 0) read_data_1 = 0;
	          else 
	          begin
	          read_data_1 = Regfile[read_addr_1];
	          //$display("read_addr_1=%d,read_data_1=%h",read_addr_1,read_data_1);
	          end
	        end
	//assign read_data_2 = Regfile[read_addr_2];
        always @(read_data_2 or Regfile[read_addr_2])
	        begin
	          if (read_addr_2 == 0) read_data_2 = 0;
	          else 
	          begin
	          read_data_2 = Regfile[read_addr_2];
	          //$display("read_addr_2=%d,read_data_2=%h",read_addr_2,read_data_2);
	          end
	        end
	always @(posedge clk)
	        begin
		      if (RegWrite == 1'b1)
		         begin 
		             Regfile[write_addr] = write_data;
		             $display("Rigister File write_addr=%d write_data=%d",write_addr,write_data);
		         end
	        end
endmodule

module holding_reg(output_data, input_data, write, clk, reset);
  // data size
  parameter N = 32;
  // inputs
  input [N-1:0] input_data;
  input	write, clk, reset;

  // outputs
  output [N-1:0] output_data;

  // Register content and output assignment
  reg [N-1:0] content;
    // update regisiter contents
  always @(posedge clk or posedge reset) 
  begin
    if (reset) 
    begin
      content <= 0;
    end
    else if (write) 
    begin
      content <= input_data;
    end
  end
  assign output_data = content;
endmodule

module Mux_32_bit (in0, in1, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module shift_left_2 (sign_in, sign_out);
	input [31:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[31:2]=sign_in[29:0];
	assign sign_out[1:0]=2'b00;
endmodule

module concate(PC_in,IR_in,PC_out);
    input [3:0] PC_in;
    input [27:0] IR_in;
    output[31:0] PC_out;
    assign PC_out={PC_in, IR_in};
endmodule

module Mux4_32_bit (in0, in1,in2, in3, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1,in2,in3;
	output [N-1:0] mux_out;
	input [1:0]select;
	assign mux_out = select[1]? (select[0]?in3: in2):(select[0]?in1:in0);
endmodule

module shift_left_2_28bit (sign_in, sign_out);
	input [25:0] sign_in;
	output [27:0] sign_out;
	assign sign_out={2'b00,sign_in};
endmodule

module Data_Memory (clk,addr, write_data, read_data, MemRead, MemWrite);
    input [31:0] addr;
    input [31:0] write_data;
    output [31:0] read_data;
    input MemRead, MemWrite,clk;
    reg [31:0] DMemory [63:0];
    integer k;
    initial begin
        for (k=0; k<64; k=k+1)
            begin
                DMemory[k] = 32'b0;
            end
        //sw  $s1, 0x02($s2)	    //	Memory[$s2+0x02] = $s1
        DMemory[0] = 32'b10101110010100010000000000000010;       
        
        //add $s4,  $s2, $s3	    //	$s4 = $s2 + $s3  => R20=0x90 
        DMemory[4] = 32'b00000010010100111010000000100000;
        
        
        //add $s5 $t0 $t1	     
        DMemory[8] = 32'b00000001000010011010100000100000;
        
        
        //sub $s1, $s2, $s3	    //	$s1 = $s2 â€“ $s3  => R17=0x22 
        DMemory[12] = 32'b00000010010100111000100000100010;
        
        //sw  $s1, 0x02($s2)	    //	Memory[$s2+0x02] = $s1
        DMemory[16] = 32'b10101110010100010000000000000010;
        
        
        //lw $s1, 0x02($s2)	        //	$s1 = Memory[$s2+0x02] 
        DMemory[20] = 32'b10001110010100010000000000000010;
        
        
        //beq $t2,$t3, End      //beq $t2,$t3, 0x03
        DMemory[24] = 32'b00010001010010110000000000000011;
        
        //addi $s7, $zero, 0x10
        DMemory[28] = 32'b00100000000101110000000000010000;
                //j 0x00
        DMemory[32] = 32'b00001000000000000000000000000000;
        //addi $s2, $zero, 0x55 //  load immediate value 0x55 to register $s2
        DMemory[36] = 32'b00100000000100100000000001010101;
        //addi $s3, $zero, 0x22 //  load immediate value 0x22 to register $s3
        DMemory[40] = 32'b00100000000100110000000000100010;
        //addi $s5, $zero, 0x77 //  load immediate value 0x77 to register $s5
        DMemory[44] = 32'b00100000000101010000000001110111;
        end
        
    assign read_data = (MemRead) ? DMemory[addr] : 32'bx;
    
    always @(posedge clk)
        begin
            if (MemWrite)
            begin
               DMemory[addr] = write_data;
               $display("Data memory write_addr=%d write_data=%d",addr,write_data);
            end
        end
endmodule
