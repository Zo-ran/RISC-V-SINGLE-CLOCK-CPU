module lab12(

	//////////// CLOCK //////////
	input 		          		CLOCK2_50,
	input 		          		CLOCK3_50,
	input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// Seg7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	output		     [6:0]		HEX4,
	output		     [6:0]		HEX5,

	//////////// VGA //////////
	output		          		VGA_BLANK_N,
	output		     [7:0]		VGA_B,
	output		          		VGA_CLK,
	output		     [7:0]		VGA_G,
	output		          		VGA_HS,
	output		     [7:0]		VGA_R,
	output		          		VGA_SYNC_N,
	output		          		VGA_VS,

	//////////// PS2 //////////
	inout 		          		PS2_CLK,
	inout 		          		PS2_CLK2,
	inout 		          		PS2_DAT,
	inout 		          		PS2_DAT2
	);

	// instruct mem
	wire [31:0] imemaddr;
	wire [31:0] imemdataout;
	wire imemclk;
	wire CPU_CLK;
    wire TIME_CLK;
    reg [31:0] TIME_CNT;
	reg [31:0] bufferout;
	wire kbd_rden;
	assign kbd_rden=(dmemaddr[31:20] == 12'h003) ? 1'b1 : 1'b0;
	

	// data mem
	always @(*) begin
		case (dmemaddr[31:20])
		12'h001: tr_data_out = dmemdataout;
		12'h003: tr_data_out = bufferout;
		12'h004: tr_data_out = vga_reg;
        12'h005: tr_data_out = TIME_CNT;
		endcase
	end

	always @(posedge dmemrdclk)
	begin
		if(kbd_rden)
		begin
			if (r_ptr==w_ptr) 
				bufferout<=32'h0;
			else
			begin
				bufferout<={24'h0, buffer[r_ptr]};
				r_ptr<=r_ptr+1;
			end
		end
	end

    always @(posedge TIME_CLK)
        TIME_CNT <= TIME_CNT + 1'b1;
			
	wire [31:0] dmemaddr;
	wire [31:0] dmemdatain;
	wire [31:0] dmemdataout;
	wire dmemwrclk;
	wire dmemrdclk;
	wire [2:0] dmemop;
	wire dmemwe;
	wire [31:0] dbgdata;
	reg [31:0] tr_data_out;

	imem my_imem(imemaddr[17:2], imemclk, imemdataout);
	rv32is cpu(CPU_CLK, SW[0], imemaddr, imemdataout, imemclk, dmemaddr, tr_data_out, dmemdatain, dmemrdclk, dmemwrclk, dmemop, dmemwe, dbgdata);
	dmem my_dmem(dmemaddr, dmemdataout, dmemdatain, dmemrdclk, dmemwrclk, dmemop, dmemwe);

	// VGA
	reg [31:0] vga_reg;
    reg led_reg [0:9];
	wire vreg_en;
    wire [11:0] vga_data;
    wire [11:0] line;
    wire [9:0] h_addr, v_addr;
    wire [3:0] red, green, blue;
    wire [11:0] rd_addr;
	wire [7:0] rd_ascii_key;
	wire vga_en;
    wire led_en;
    reg [11:0] wr_addr;

	assign rd_addr = ((((v_addr >> 4) + vga_reg) % 64) << 6) + h_addr / 9;//change
	assign vga_data = (v_addr < 480 && h_addr < 576) ? ((line[h_addr % 9]) ? 12'hfff : 12'h0) : 12'h0; 
    assign VGA_R = {red, 4'b0};
    assign VGA_B = {blue, 4'b0};
    assign VGA_G = {green, 4'b0};
    assign VGA_SYNC_N = 0;
	assign vga_en = (dmemaddr[31:20] == 12'h002) ? dmemwe : 1'b0;
	assign vreg_en = (dmemaddr[31:20] == 12'h004) ? dmemwe : 1'b0;
    assign led_en = (dmemaddr[31:20] == 12'h006) ? dmemwe : 1'b0;
    assign LEDR[0] = led_reg[0];
    assign LEDR[1] = led_reg[1];
    assign LEDR[2] = led_reg[2];
    assign LEDR[3] = led_reg[3];
    assign LEDR[4] = led_reg[4];
    assign LEDR[5] = led_reg[5];
    assign LEDR[6] = led_reg[6];
    assign LEDR[7] = led_reg[7];
    assign LEDR[8] = led_reg[8];
    assign LEDR[9] = led_reg[9];
	clkgen #(25000000) my_vgaclk(CLOCK_50, SW[0], 1'b1, VGA_CLK);
	clkgen #(250000) my_cpuclk(CLOCK_50, SW[0], 1'b1, CPU_CLK);
    clkgen #(1) my_timeclk(CLOCK_50, SW[0], 1'b1, TIME_CLK);
	vga_font font({rd_ascii_key[7:0], v_addr[3:0]}, CLOCK_50, line);
    vga_ctrl vga(VGA_CLK, 1'b0, vga_data, h_addr, v_addr, VGA_HS, VGA_VS, VGA_BLANK_N, red, green, blue);
    VRAM video(dmemdatain[7:0], rd_addr, CLOCK_50, dmemaddr[11:0], dmemwrclk, vga_en, rd_ascii_key);

	always @(posedge dmemwrclk) begin
		if (vreg_en)
			vga_reg <= dmemdatain;
		else
			vga_reg <= vga_reg;
	end

    always @(posedge dmemwrclk)
        if (led_en) 
            led_reg[dmemaddr[3:0]] = ~led_reg[dmemaddr[3:0]];

	// Keyboard
    wire ready;
    reg nextdata_n;
    wire overflow;
    wire [7:0] keydata;
	wire [7:0] wr_ascii_key;
    reg released;
    reg shift;
    reg capslock;
	reg [7:0] buffer [15:0];
	reg [3:0] r_ptr;
	reg [3:0] w_ptr;

initial begin
    nextdata_n = 1;
    capslock = 0;
    shift = 0;
    released = 0;
end
		
    scancode_rom myrom(keydata, capslock, shift, wr_ascii_key);
    ps2_keyboard mykey(CLOCK_50, KEY[1], PS2_CLK, PS2_DAT, keydata, ready, nextdata_n, overflow);
    always @(posedge CPU_CLK) begin//change
        if (KEY[1] == 0) begin
            nextdata_n <= 1;
            capslock <= 0;
            shift <= 0;
            released <= 0;
        end
        else begin
            if (nextdata_n == 0)
                nextdata_n <= 1;
            else if (ready) begin
                nextdata_n <= 0;
                if (keydata != 8'hf0) begin
                    if (released) begin
                        released <= 0;
                        if (keydata == 8'h12)
                            shift <= 0;
                    end
                    else begin
                        if (keydata == 8'h58)
                            capslock <= ~capslock;
                        else if (keydata == 8'h12)
                            shift <= 1;
                        else begin
                            buffer[w_ptr] <= wr_ascii_key;
                            w_ptr <= w_ptr + 1'b1;
                        end
                    end
                end
                else 
                    released <= 1;
            end
            else
                nextdata_n <= 1;
        end
    end	

endmodule

module dmem(
	input  [31:0] raw_addr,
	output reg [31:0] dataout,
	input  [31:0] datain,
	input  rdclk,
	input  wrclk,
	input [2:0] memop,
	input we);
	reg [3:0] byte_en;
	wire [31:0] tr_dmem_dataout;
	tr_dmem my_tr_dmem(byte_en, datain << (8 * raw_addr[1:0]), raw_addr[16:2], rdclk, raw_addr[16:2], wrclk, (raw_addr[31:20] == 12'h001) ? we : 1'b0, tr_dmem_dataout);
	always @(*) begin
		case (memop)
			3'b000: begin
				case (raw_addr[1:0]) 
					2'b00: dataout = tr_dmem_dataout[7] ? {24'hffffff, tr_dmem_dataout[7:0]} : {24'h0, tr_dmem_dataout[7:0]};
					2'b01: dataout = tr_dmem_dataout[15] ? {24'hffffff, tr_dmem_dataout[15:8]} : {24'h0, tr_dmem_dataout[15:8]};
					2'b10: dataout = tr_dmem_dataout[23] ? {24'hffffff, tr_dmem_dataout[23:16]} : {24'h0, tr_dmem_dataout[23:16]};
					2'b11: dataout = tr_dmem_dataout[31] ? {24'hffffff, tr_dmem_dataout[31:24]} : {24'h0, tr_dmem_dataout[31:24]};
				endcase
			end
			3'b010: dataout = tr_dmem_dataout;
			3'b100: begin
				case (raw_addr[1:0]) 
					2'b00: dataout = {24'h0, tr_dmem_dataout[7:0]};
					2'b01: dataout = {24'h0, tr_dmem_dataout[15:8]};
					2'b10: dataout = {24'h0, tr_dmem_dataout[23:16]};
					2'b11: dataout = {24'h0, tr_dmem_dataout[31:24]};
				endcase
			end
			3'b001: begin
				case (raw_addr[1]) 
					1'b0: dataout = tr_dmem_dataout[15] ? {16'hffff, tr_dmem_dataout[15:0]} : {16'h0, tr_dmem_dataout[15:0]};
					1'b1: dataout = tr_dmem_dataout[31] ? {16'hffff, tr_dmem_dataout[31:16]} : {16'h0, tr_dmem_dataout[31:16]};
				endcase
			end
			3'b101: begin
				case (raw_addr[1]) 
					1'b0: dataout = {16'h0, tr_dmem_dataout[15:0]};
					1'b1: dataout = {16'h0, tr_dmem_dataout[31:16]};
				endcase
			end
		endcase
	end
	always @(*) begin
		case (memop)
			3'b010: byte_en = 4'b1111;
			3'b000: begin
				case (raw_addr[1:0]) 
					2'b00: byte_en = 4'b0001;
					2'b01: byte_en = 4'b0010;
					2'b10: byte_en = 4'b0100;
					2'b11: byte_en = 4'b1000;
				endcase
			end
			3'b001: begin
				case (raw_addr[1]) 
					1'b0: byte_en = 4'b0011;
					1'b1: byte_en = 4'b1100;
				endcase
			end
		endcase
	end

endmodule

module rv32is(
	input 	clock,
	input 	reset,
	output [31:0] imemaddr,
	input  [31:0] imemdataout,
	output 	imemclk,
	output [31:0] dmemaddr,
	input  [31:0] dmemdataout,
	output [31:0] dmemdatain,
	output 	dmemrdclk,
	output	dmemwrclk,
	output [2:0] dmemop,
	output	dmemwe,
	output [31:0] dbgdata
);
	wire true_clock = clock;
	// mem
	assign dmemop = MemOP;
	assign dmemrdclk = true_clock;
	assign dmemwrclk = ~true_clock;
	assign dmemwe = MemWr;
	assign dmemdatain = busB;
	assign dmemaddr = aluresult;

	// PC related
function [31:0] genPC;

	input [2:0] Branch;
	input zero;
	input less;
	input [31:0] PC;
	input [31:0] imm;
	input [31:0] busA;

begin
	case (Branch)
	3'b000: genPC = PC + 32'h4; 
	3'b001: genPC = PC + imm; 
	3'b010: genPC = busA + imm; 	
	3'b100: genPC = (zero) ? (PC + imm) : (PC + 32'h4);
	3'b101: genPC = (zero) ? (PC + 32'h4) : (PC + imm);
	3'b110: genPC = (less) ? (PC + imm) : (PC + 32'h4);
	3'b111: genPC = (less) ? (PC + 32'h4) : (PC + imm);
	endcase
end
	
endfunction
	wire [31:0] NextPC;
	reg [31:0] PC;
	assign imemaddr = (reset) ? 0 : NextPC;
	assign dbgdata = (reset) ? 0 : PC;
	assign imemclk = ~true_clock;
	always @(negedge true_clock) begin
		if (reset)
			PC <= 0;
		else 
			PC <= NextPC;
	end		
	assign NextPC = genPC(Branch, zero, less, PC, imm, busA);

	// data
	wire [31:0] instr;
	wire [6:0] opcode;
	wire [4:0] rs1;
	wire [4:0] rs2;
	wire [4:0] rd;
	wire [2:0] func3;
	wire [6:0] func7;
	assign instr = imemdataout;
	assign opcode = instr[6:0];
	assign rs1 = instr[19:15];
	assign rs2 = instr[24:20];
	assign rd = instr[11:7];
	assign func3 = instr[14:12];
	assign func7 = instr[31:25];

	// imm
function [31:0] selimm;

	input [2:0] ExtOP;
	input [31:0] immI; 
	input [31:0] immU; 
	input [31:0] immS; 
	input [31:0] immB; 
	input [31:0] immJ; 
begin
	case (ExtOP)
	3'b000: selimm = immI;
	3'b001: selimm = immU;
	3'b010: selimm = immS;
	3'b011: selimm = immB;
	3'b100: selimm = immJ;
	default: selimm = 32'h0;
	endcase
end

endfunction

	wire [31:0] immI; 
	wire [31:0] immU; 
	wire [31:0] immS; 
	wire [31:0] immB; 
	wire [31:0] immJ; 
	wire [31:0] imm;
	assign immI = {{20{instr[31]}}, instr[31:20]};
	assign immU = {instr[31:12], 12'b0};
	assign immS = {{20{instr[31]}}, instr[31:25], instr[11:7]};
	assign immB = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
	assign immJ = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
	assign imm = selimm(ExtOP, immI, immU, immS, immB, immJ);


	// control signal
	wire [2:0] ExtOP;
	wire RegWr;
	wire ALUAsrc;
	wire [1:0] ALUBsrc;
	wire [3:0] ALUctr;
	wire [2:0] Branch;
	wire MemtoReg;
	wire MemWr;
	wire [2:0] MemOP;
	contrgen myctr(opcode, func3, func7, ExtOP, RegWr, ALUAsrc, ALUBsrc, ALUctr, Branch, MemtoReg, MemWr, MemOP);

	// regs
	wire [31:0] busA;
	wire [31:0] busB;
	wire [31:0] busW;
	assign busW = (MemtoReg) ? dmemdataout : aluresult;
	regfile myregfile(rs1, rs2, rd, busW, RegWr, ~true_clock, busA, busB);

	// alu
function [31:0] alub;

	input [1:0] ALUBsrc;
	input [31:0] busB;
	input [31:0] imm;
begin
	case (ALUBsrc)
		2'b00: alub = busB;
		2'b01: alub = imm;
		2'b10: alub = 32'h4;
		default: alub = 32'h0;
	endcase
end

endfunction

	wire [31:0] dataa;
	wire [31:0] datab;
	wire less;
	wire zero;
	wire [31:0] aluresult;
	assign dataa = (ALUAsrc) ? PC : busA;
	assign datab = alub(ALUBsrc, busB, imm);
	alu myalu(dataa, datab, ALUctr, less, zero, aluresult);

endmodule

module alu(
	input [31:0] dataa,
	input [31:0] datab,
	input [3:0]  ALUctr,
	output less,
	output zero,
	output reg [31:0] aluresult);

//add your code here
wire cf;
wire of;
wire zf;
wire [31:0] datatemp;

assign less=aluresult[0];
assign zero=(ALUctr==4'b0010||ALUctr==4'b1010)?zf:~(|aluresult);

alu_s my_alu(dataa,datab,ALUctr,datatemp,cf,zf,of);

always @(*)
begin
	case(ALUctr)
		4'b0000:aluresult=datatemp;
		4'b1000:aluresult=datatemp;
		4'b0001,4'b1001:aluresult=(dataa<<datab[4:0]);
		4'b0101:aluresult=(dataa>>datab[4:0]);
		4'b1101:aluresult=(($signed(dataa))>>>datab[4:0]);
		4'b0010:begin
					if(dataa[31]==1&&datab[31]==0)aluresult=1;
					else if(dataa[31]==0&&datab[31]==1)aluresult=0;
					else
					begin
						if(dataa<datab)aluresult=1;
						else aluresult=0;
					end
				end
		4'b1010:begin
					if(dataa<datab)aluresult=1;
					else aluresult=0;
				end
		4'b0011,4'b1011:aluresult=datab;
		4'b0100,4'b1100:aluresult=dataa^datab;
		4'b0110,4'b1110:aluresult=dataa|datab;
		4'b0111,4'b1111:aluresult=dataa&datab;
	endcase
end

endmodule

module alu_s( input [31:0] A,
			  input [31:0] B,
			  input [3:0] ALUctr,
			  output reg [31:0] F,
			  output reg cf,
			  output reg zero,
			  output reg of
);

wire [31:0] f;
wire _cf,_zero,_of;
reg addsub;
always @(*)
begin
	if(ALUctr==0)addsub=1'b0;
	else addsub=1'b1;
end

adder add(A,B,addsub,f,_cf,_zero,_of);

wire greater=(A[31]==1'b0&B[31]==1'b1)|(f[31]^_of==0&&_zero==0);

always @(*)begin
	case(ALUctr[2:0])
		0,1,2:begin F=f;cf=_cf;zero=_zero;of=_of;end
		3:begin F=A&B;cf=1'b0;zero=~(|F);of=1'b0;end
		4:begin F=A|B;cf=1'b0;zero=~(|F);of=1'b0;end
		5:begin F=A^B;cf=1'b0;zero=~(|F);of=1'b0;end
		6:begin F={3'b0,greater};cf=1'b0;zero=~(|F);of=1'b0;end
		7:begin F={3'b0,_zero};cf=1'b0;zero=_zero;of=1'b0;end
	endcase
end
endmodule

module adder(
	input  [31:0] A,
	input  [31:0] B,
	input  addsub,
	output [31:0] F,
	output cf,
	output zero,
	output of
	);

// add your code here
wire [31:0] b;
wire out_c;
assign b=B^{32{addsub}};
assign {out_c,F}=A+b+addsub;

assign cf=out_c^addsub;

assign of = (A[31] == b[31]) && (F[31] != A[31]);

assign zero=~(|F);

endmodule

module contrgen (
	input [6:0] opcode,
	input [2:0] func3,
	input [6:0] func7,
	output reg [2:0] ExtOP,
	output reg RegWr,
	output reg ALUAsrc,
	output reg [1:0] ALUBsrc,
	output reg [3:0] ALUctr,
	output reg [2:0] Branch,
	output reg MemtoReg,
	output reg MemWr,
	output reg [2:0] MemOP
);

	always @(*) begin
		case (opcode[6:2])
		5'b01101: begin
			ExtOP <= 3'b001;
			RegWr <= 1'b1;
			Branch <= 3'b0;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUBsrc <= 2'b01;
			ALUctr <= 4'b0011;
			MemOP <= 3'b111;
		end 
		5'b00101: begin
			ExtOP <= 3'b001;
			RegWr <= 1'b1;
			Branch <= 3'b0;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b1;
			ALUBsrc <= 2'b01;
			ALUctr <= 4'b0000;
			MemOP <= 3'b111;
		end
		5'b00100: begin
			ExtOP <= 3'b000;
			RegWr <= 1'b1;
			Branch <= 3'b0;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b0;
			ALUBsrc <= 2'b01;
			MemOP <= 3'b111;
			case (func3)
			3'b000: ALUctr <= 4'b0000;
			3'b010: ALUctr <= 4'b0010;
			3'b011: ALUctr <= 4'b1010;	
			3'b100: ALUctr <= 4'b0100;	
			3'b110: ALUctr <= 4'b0110;	
			3'b111: ALUctr <= 4'b0111;	
			3'b001: if (func7[5] == 0) ALUctr <= 4'b0001;
			3'b101: ALUctr <= (func7[5]) ? 4'b1101 : 4'b0101;
			endcase
		end
		5'b01100: begin
			RegWr <= 1'b1;
			Branch <= 3'b0;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b0;
			ALUBsrc <= 2'b00;
			MemOP <= 3'b111;
			case (func3)
			3'b000: ALUctr <= (func7[5]) ? 4'b1000 : 4'b0000;
			3'b001: if (func7[5] == 0) ALUctr <= 4'b0001;
			3'b010: if (func7[5] == 0) ALUctr <= 4'b0010;
			3'b011: if (func7[5] == 0) ALUctr <= 4'b1010;
			3'b100: if (func7[5] == 0) ALUctr <= 4'b0100;
			3'b101: ALUctr <= (func7[5]) ? 4'b1101 : 4'b0101;
			3'b110: if (func7[5] == 0) ALUctr <= 4'b0110;
			3'b111: if (func7[5] == 0) ALUctr <= 4'b0111;
			endcase
		end
		5'b11011: begin
			ExtOP <= 3'b100;
			RegWr <= 1'b1;
			Branch <= 3'b001;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b1;
			ALUBsrc <= 2'b10;
			ALUctr <= 4'b0000;
			MemOP <= 3'b111;
		end
		5'b11001: begin
			ExtOP <= 3'b000;
			RegWr <= 1'b1;
			Branch <= 3'b010;
			MemtoReg <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b1;
			ALUBsrc <= 2'b10;
			ALUctr <= 4'b0000;
			MemOP <= 3'b111;
		end
		5'b11000: begin
			ExtOP <= 3'b011;
			RegWr <= 1'b0;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b0;
			ALUBsrc <= 2'b0;
			MemOP <= 3'b111;
			case (func3)
			3'b000: begin
				Branch <= 3'b100;
				ALUctr <= 4'b0010;
			end
			3'b001: begin
				Branch <= 3'b101;
				ALUctr <= 4'b0010;
			end
			3'b100: begin
				Branch <= 3'b110;
				ALUctr <= 4'b0010;
			end
			3'b101: begin
				Branch <= 3'b111;
				ALUctr <= 4'b0010;
			end
			3'b110: begin
				Branch <= 3'b110;
				ALUctr <= 4'b1010;
			end
			3'b111: begin
				Branch <= 3'b111;
				ALUctr <= 4'b1010;
			end
			endcase
		end
		5'b00000: begin
			ExtOP <= 3'b000;
			RegWr <= 1'b1;
			Branch <= 3'b000;
			MemtoReg <= 1'b1;
			MemWr <= 1'b0;
			ALUAsrc <= 1'b0;
			ALUBsrc <= 2'b01;
			ALUctr <= 4'b0000;
			MemOP <= func3;
		end
		5'b01000: begin
			ExtOP <= 3'b010;
			RegWr <= 1'b0;
			Branch <= 3'b000;
			MemWr <= 1'b1;
			ALUAsrc <= 1'b0;
			ALUBsrc <= 2'b01;
			ALUctr <= 4'b0000;
			MemOP <= func3;
		end
		endcase
	end

endmodule

module regfile(
	input  [4:0]  ra,
	input  [4:0]  rb,
	input  [4:0]  rw,
	input  [31:0] wrdata,
	input  regwr,
	input  wrclk,
	output [31:0] outa,
	output [31:0] outb
);
	
	reg [31:0] regs[31:0];	
	assign outa = regs[ra];
	assign outb = regs[rb];
	always @ (posedge wrclk) begin
		if(regwr)
			regs[rw]<=(rw==5'b00000)?32'b0:wrdata;
	end
	
endmodule

module clkgen(
	input clkin,
	input rst,
	input clken,
	output reg clkout
	);
	parameter clk_freq=1000;
	parameter countlimit=50000000/2/clk_freq; // 自动计算计数次数
 	reg[31:0] clkcount;
 	always @ (posedge clkin)
 		if (rst) begin
 			clkcount=0;
 			clkout=1'b0;
 		end
		else begin
			if (clken) begin
				clkcount=clkcount+1;
				if (clkcount >= countlimit) begin
					clkcount=32'd0;
					clkout=~clkout;
				end
				else
					clkout=clkout;
			end
			else begin
				clkcount = clkcount;
				clkout = clkout;
			end
		end
endmodule


module vga_ctrl(
   input pclk, //25MHz 时钟
   input reset, // 置位
   input [11:0] vga_data, // 上层模块提供的VGA 颜色数据
   output [9:0] h_addr, // 提供给上层模块的当前扫描像素点坐标
   output [9:0] v_addr,
   output hsync, // 行同步和列同步信号
   output vsync,
   output valid, // 消隐信号
   output [3:0] vga_r, // 红绿蓝颜色信号
   output [3:0] vga_g,
   output [3:0] vga_b
   );
   
   //640x480 分辨率下的VGA 参数设置
   parameter h_frontporch = 96;
   parameter h_active = 144;
   parameter h_backporch = 784;
   parameter h_total = 800;
   parameter v_frontporch = 2;
   parameter v_active = 35;
   parameter v_backporch = 515;
   parameter v_total = 525;
   
   // 像素计数值
   reg [9:0] x_cnt;
   reg [9:0] y_cnt;
   wire h_valid;
   wire v_valid;
   
   always @(posedge reset or posedge pclk) // 行像素计数
      if (reset == 1'b1)
         x_cnt <= 1;
      else begin
         if (x_cnt == h_total)
            x_cnt <= 1;
         else
            x_cnt <= x_cnt + 10'd1;
      end
      
   always @(posedge pclk) // 列像素计数
      if (reset == 1'b1)
         y_cnt <= 1;
      else begin
         if (y_cnt == v_total & x_cnt == h_total)
            y_cnt <= 1;
         else if (x_cnt == h_total)
            y_cnt <= y_cnt + 10'd1;
      end
      
   // 生成同步信号
   assign hsync = (x_cnt > h_frontporch);
   assign vsync = (y_cnt > v_frontporch);
   
   // 生成消隐信号
   assign h_valid = (x_cnt > h_active) & (x_cnt <= h_backporch);
   assign v_valid = (y_cnt > v_active) & (y_cnt <= v_backporch);
   assign valid = h_valid & v_valid;
   
   // 计算当前有效像素坐标
   assign h_addr = h_valid ? (x_cnt - 10'd145) : {10{1'b0}};
   assign v_addr = v_valid ? (y_cnt - 10'd36) : {10{1'b0}};
   
   // 设置输出的颜色值
   assign vga_r = vga_data[11:8];
   assign vga_g = vga_data[7:4];
   assign vga_b = vga_data[3:0];
   
endmodule

module ps2_keyboard(clk, clrn, ps2_clk, ps2_data, data, ready, nextdata_n, overflow);
    input clk,clrn,ps2_clk,ps2_data;
	 input nextdata_n;
    output [7:0] data;
    output reg ready;
    output reg overflow;     // fifo overflow  
    // internal signal, for test
    reg [9:0] buffer;        // ps2_data bits
    reg [7:0] fifo[7:0];     // data fifo
	 reg [2:0] w_ptr,r_ptr;   // fifo write and read pointers	
    reg [3:0] count;  // count ps2_data bits
    // detect falling edge of ps2_clk
    reg [2:0] ps2_clk_sync;
    
    always @(posedge clk) begin
        ps2_clk_sync <=  {ps2_clk_sync[1:0],ps2_clk};
    end

    wire sampling = ps2_clk_sync[2] & ~ps2_clk_sync[1];
    
    always @(posedge clk) begin
        if (clrn == 0) begin // reset 
            count <= 0; w_ptr <= 0; r_ptr <= 0; overflow <= 0; ready<= 0;
        end 
		else if (sampling) begin
            if (count == 4'd10) begin
                if ((buffer[0] == 0) &&  // start bit
                    (ps2_data)       &&  // stop bit
                    (^buffer[9:1])) begin      // odd  parity
                    fifo[w_ptr] <= buffer[8:1];  // kbd scan code
                    w_ptr <= w_ptr+3'b1;
                    ready <= 1'b1;
                    overflow <= overflow | (r_ptr == (w_ptr + 3'b1));
                end
                count <= 0;     // for next
            end else begin
                buffer[count] <= ps2_data;  // store ps2_data 
                count <= count + 3'b1;
            end      
        end
        if ( ready ) begin // read to output next data
				if(nextdata_n == 1'b0) //read next data
				begin
				   r_ptr <= r_ptr + 3'b1; 
					if(w_ptr==(r_ptr+1'b1)) //empty
					     ready <= 1'b0;
				end           
        end
    end

    assign data = fifo[r_ptr];
endmodule 

module scancode_rom(addr, caps, shift, outdata);
input [7:0] addr;
input caps;
input shift;
output reg [7:0] outdata;
// ascii rom
reg [7:0] ascii_tab[255:0];
reg [7:0] ascii_tab_caps[255:0];
reg [7:0] ascii_tab_shift[255:0];
reg [7:0] ascii_tab_shiftcaps[255:0];

initial begin
    $readmemh("E:/DigitalCircuitLab/lab12/scancode.txt", ascii_tab, 0, 255);
    $readmemh("E:/DigitalCircuitLab/lab12/capscode.txt", ascii_tab_caps, 0, 255);
    $readmemh("E:/DigitalCircuitLab/lab12/shiftcode.txt", ascii_tab_shift, 0, 255);
    $readmemh("E:/DigitalCircuitLab/lab12/shiftcapscode.txt", ascii_tab_shiftcaps, 0, 255);
end

always @(*)
begin
    case ({caps, shift})
        2'b00: outdata = ascii_tab[addr];
        2'b01: outdata = ascii_tab_shift[addr];
        2'b10: outdata = ascii_tab_caps[addr];
        2'b11: outdata = ascii_tab_shiftcaps[addr];
    endcase
end

endmodule