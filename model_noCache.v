//FETCH -> if_de_Register -> DECODE -> de_ex_Reguster -> EXECUTE -> ex_mem_Register -> MEMORY -> mem_wb_Register -> WRITE BACK

module instructionMemory(
    input clk,
    input [5:0] address,
    output reg [31:0] instruction
);

    reg [31:0] memory[0:8]; // Internal ROM

    initial begin
        $readmemh("program.mem", memory);
    end

    always @(posedge clk) begin
        instruction <= memory[address];
    end
endmodule


module registerFile(
    input clk,
    input writeEnable,
    input [4:0] destAdd_wb, destAdd_dataFetch, r1_add,
    input [15:0] r2_add,
    input [7:0] writeData,
    output [7:0] r1, r2, destAddVal
);

reg [7:0] registers[0:31]; //8 bit data only

initial begin
    $readmemh("register.mem", registers);
end

//read_data
assign r1 = registers[r1_add];
assign r2 = registers[r2_add[4:0]];
assign destAddVal = registers[destAdd_dataFetch];

always @(posedge clk) begin //writing into a register takes one clock cycle
    if (writeEnable && destAdd_wb != 0) begin
        registers[destAdd_wb] <= writeData;
    end
    $writememh("reg_dump.mem", registers);
end

endmodule

module storage(
    input clk,
    input [7:0] addr,
    input [7:0] data_toMemory,
    input memRead, memWrite,
    output reg [7:0] data_fromMemory
);

reg [7:0] DataMemory [0:255];

always @(posedge clk) begin
    if (memWrite == 1) begin
        DataMemory[addr] <= data_toMemory;
    end
    if (memRead == 1) begin
        data_fromMemory <= DataMemory[addr];
    end

    $writememh("memory_dump.mem", DataMemory);
end
endmodule

module decode(
    input clk,
    input [31:0] instruction,
    output reg [3:0] aluTask,
    output reg [4:0] operand_1, destAddrReg, shift,
    output reg [15:0] operand_2,
    output reg regWrite, memRead, memWrite, aluSrc, memToReg //flags
);

/*
Operation   Description	            Typical ALU Control Code
ADD         Addition	            3'b001
SUB         Subtraction	            3'b010
AND         Bitwise AND	            3'b011
OR          Bitwise OR	            3'b100
XOR         Bitwise XOR 	        3'b101
SLL         Shift Left Logical  	3'b110
SRL         Shift Right Logical	    3'b111
SLT         Set on Less Than (signed) 3'b1000
*/

//R type instruction: Opcode (6bits), Reg1 (5 bits), Reg2 (5 bits), DestReg (5 bits), Shift (5 bits), Function (6bits)
//I type instruction: Opcode (6bits), Reg1 (5 bits), Reg2 (5 bits), Immediate (16 bits)

reg [5:0] opcode, funct;

always @(*) begin
    opcode <= (instruction >> 26);

    case((instruction >> 26))
        6'b000000: begin
            regWrite <= 1'b1;
            memRead <= 1'b0;
            memWrite <= 1'b0;
            memToReg <= 1'b0;
            aluSrc <= 1'b0;

            //ADD R1, R2, R3: 000000 00010 00011 00001 00000 000001

            funct <= (instruction & 6'b111111);
            aluTask <= funct[3:0];
            operand_1 <= ((instruction >> 21) & 5'b11111);
            operand_2 <= ((instruction >> 16) & 5'b11111);
            destAddrReg <= ((instruction >> 11) & 5'b11111);
            shift <= ((instruction >> 6) & 5'b11111);

        end
        6'b000001, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110, 6'b000111, 6'b001000: begin
            regWrite <= 1'b1;
            memRead <= 1'b0;
            memWrite <= 1'b0;
            memToReg <= 1'b0;
            aluSrc <= 1'b1;

            aluTask <= instruction[29:26];

            //aluTask <= (instruction >> 26)[3:0];

            //ADDI R1, R2, 50: 000001 00010 00001 0000000000110010

            destAddrReg <= ((instruction >> 16) & 5'b11111);
            operand_1 <= ((instruction >> 21) & 5'b11111);
            operand_2 <= (instruction & 16'hFFFF);
        end
        6'b001001: begin //load word
            regWrite <= 1'b1;
            memRead <= 1'b1;
            memWrite <= 1'b0;
            memToReg <= 1'b1;
            aluSrc <= 1'b1;

            aluTask <= 4'b0001;

            //LW R1, 100(R2): 001001 (op), 00010 (R2), 00001 (R1), 0000000001100100 (#100)
            destAddrReg <= ((instruction >> 16) & 5'b11111);
            operand_1 <= ((instruction >> 21) & 5'b11111);
            operand_2 <= (instruction & 16'hFFFF);
        end
        6'b001010: begin //store word
            memRead <= 1'b0;
            memWrite <= 1'b1;
            regWrite <= 1'b0;
            memToReg <= 1'b0;
            aluSrc <= 1'b1;

            aluTask <= 4'b0001;

            //SW R1, 100(R2): 001010 (op), 00010 (R2), 00001 (R1), 0000000001100100 (#100)

            destAddrReg <= ((instruction >> 16) & 5'b11111);
            operand_1 <= ((instruction >> 21) & 5'b11111);
            operand_2 <= (instruction & 16'hFFFF);
        end

        default : begin
        regWrite <= 0;
        memRead <= 0;
        memWrite <= 0;
        memToReg <= 0;
        aluSrc <= 0;
        aluTask <= 4'b0000;
    end
    endcase
end
endmodule

module aluSrcMuxer(
    input aluSrc,
    input [7:0] r2_register,
    input [15:0] intermediate,

    output reg [15:0] aluInput2
);

always @(*) begin
    case (aluSrc)
        1'b0: begin
            //aluInput2 <= {{9{r2_register[6]}}, r2_register};
            aluInput2 <= {8'b0, r2_register};
        end
        1'b1: begin
            aluInput2 <= intermediate;
        end
    endcase

end

endmodule

module ALU(
    input clk,
    input [3:0] aluTask,
    input [7:0] operand1,
    input [15:0] operand2,
    input [4:0] shamt,
    output reg [7:0] result
);

/*
Operation   Description	            Typical ALU Control Code
ADD         Addition	            4'b001
SUB         Subtraction	            4'b010
AND         Bitwise AND	            4'b011
OR          Bitwise OR	            4'b100
XOR         Bitwise XOR 	        4'b101
SLL         Shift Left Logical  	4'b110
SRL         Shift Right Logical	    4'b111
SLT         Set on Less Than (signed) 4'b1000
*/

always @(*) begin
    case (aluTask)
        4'b0001: begin
            result <= operand1 + operand2[7:0];
        end

        4'b0010: begin
            result <= operand1 - operand2[7:0];
        end

        4'b0011: begin
            result <= operand1 & operand2[7:0];
        end

        4'b0100: begin
            result <= operand1 | operand2[7:0];
        end

        4'b0101: begin
            result <= operand1 ^ operand2[7:0];
        end

        4'b0110: begin
            result <= operand1[7:0] << shamt;
        end

        4'b0111: begin
            result <= operand1[7:0] >> shamt;
        end

        4'b1000: begin
            result <= (operand1 < operand2) ? 1 : 0;
        end 

        default: result <= 8'b0;
    endcase
end
endmodule

//IF/ID: Holds fetched instruction + PC.

module ifid_register(
    input clk,
    input [31:0] instruction,
    output reg [31:0] instr_ifid
);

always @(posedge clk) begin
    instr_ifid <= instruction;
end
endmodule

//ID/EX: Holds decoded signals + register values.

module idex_register(
    input clk,
    input [3:0] aluTask,
    input [4:0] operand_1, destAddrReg, shift,
    input [15:0] operand_2,
    input regWrite, memRead, memWrite, aluSrc, memToReg,

    output reg [3:0] aluTask_idex,
    output reg [4:0] operand_1_idex, destAddrReg_idex, shift_idex,
    output reg [15:0] operand_2_idex,
    output reg regWrite_idex, memRead_idex, memWrite_idex, aluSrc_idex, memToReg_idex //flags
);

always @(posedge clk) begin
    aluTask_idex <= aluTask;
    operand_1_idex <= operand_1;
    destAddrReg_idex <= destAddrReg;
    shift_idex <= shift;
    operand_2_idex <= operand_2;
    regWrite_idex <= regWrite;
    memRead_idex <= memRead;
    memWrite_idex <= memWrite;
    aluSrc_idex <= aluSrc;
    memToReg_idex <= memToReg;
end
endmodule

//EX/MEM: Holds ALU result + memory control.

module exmem_register (
    input clk,
    input [7:0] alu_result, r2, destAddVal,
    input [4:0] dest_addr, // for writing back into a register
    input regWrite, memRead, memWrite, memToReg,

    output reg [7:0] alu_result_exmem, r2_exmem, destAddVal_exmem,
    output reg [4:0] dest_addr_exmem,
    output reg regWrite_exmem, memRead_exmem, memWrite_exmem, memToReg_exmem
);

always @(posedge clk) begin
    alu_result_exmem <= alu_result;
    r2_exmem <= r2;
    destAddVal_exmem <= destAddVal;
    dest_addr_exmem <= dest_addr;
    regWrite_exmem <= regWrite;
    memRead_exmem <= memRead;
    memWrite_exmem <= memWrite;
    memToReg_exmem <= memToReg;
end
endmodule

//MEM/WB: Holds memory read data or ALU result.

module memwb_register(
    input clk,
    input [7:0] mem_data, alu_result,
    input [4:0] dest_addr,
    input regWrite, memToReg,

    output reg [7:0] mem_data_memwb, alu_result_memwb,
    output reg [4:0] dest_addr_memwb,
    output reg regWrite_memwb, memToReg_memwb
);

always @(posedge clk) begin
    mem_data_memwb <= mem_data;
    alu_result_memwb <= alu_result;
    dest_addr_memwb <= dest_addr;
    regWrite_memwb <= regWrite;
    memToReg_memwb <= memToReg;
end

endmodule

module wbDataMux(
    input memToReg_memwb,
    input [7:0] mem_data_memwb, alu_result_memwb,

    output reg [7:0] dataToWrite
);

always @(*) begin
    case (memToReg_memwb)
        1'b0: begin
            //aluInput2 <= {{9{r2_register[6]}}, r2_register};
            dataToWrite = alu_result_memwb;
        end
        1'b1: begin
            dataToWrite = mem_data_memwb;
        end
    endcase
end

endmodule

// Processor code

module top_level_processor(input clk);

reg [5:0] pc = 0;
wire [31:0] instruction;
wire [31:0] instr_ifid;

wire [3:0] aluTask;
wire [4:0] operand_1, destAddrReg, shift;
wire [15:0] operand_2;
wire regWrite, memRead, memWrite, aluSrc, memToReg; //flags

wire [3:0] aluTask_idex;
wire [4:0] operand_1_idex, destAddrReg_idex, shift_idex;
wire [15:0] operand_2_idex;
wire regWrite_idex, memRead_idex, memWrite_idex, aluSrc_idex, memToReg_idex; //flags

wire [7:0] r1_reg, r2_reg, destAddVal_reg;

wire [15:0] aluInput2;

wire [7:0] alu_result;

wire [7:0] alu_result_exmem, r2_exmem, destAddVal_exmem;
wire [4:0] dest_addr_exmem;
wire regWrite_exmem, memRead_exmem, memWrite_exmem, memToReg_exmem;

wire [7:0] data_fromMemory;

wire [7:0] mem_data_memwb, alu_result_memwb;
wire [4:0] dest_addr_memwb;
wire regWrite_memwb, memToReg_memwb;

wire [7:0] dataToWrite;

instructionMemory im(
    .clk(clk),
    .address(pc),
    .instruction(instruction)
);

ifid_register iiReg(
    .clk(clk),
    .instruction(instruction),
    .instr_ifid(instr_ifid)
);

decode dec(
    .clk(clk),
    .instruction(instr_ifid),
    .aluTask(aluTask),
    .operand_1(operand_1),
    .destAddrReg(destAddrReg),
    .shift(shift),
    .operand_2(operand_2),
    .regWrite(regWrite),
    .memRead(memRead),
    .memWrite(memWrite),
    .aluSrc(aluSrc),
    .memToReg(memToReg)
);

idex_register ieReg(
    .clk(clk),
    .aluTask(aluTask),
    .operand_1(operand_1),
    .destAddrReg(destAddrReg),
    .shift(shift),
    .operand_2(operand_2),
    .regWrite(regWrite),
    .memRead(memRead),
    .memWrite(memWrite),
    .aluSrc(aluSrc),
    .memToReg(memToReg),

    .aluTask_idex(aluTask_idex),
    .operand_1_idex(operand_1_idex), 
    .destAddrReg_idex(destAddrReg_idex), 
    .shift_idex(shift_idex),
    .operand_2_idex(operand_2_idex),
    .regWrite_idex(regWrite_idex), 
    .memRead_idex(memRead_idex), 
    .memWrite_idex(memWrite_idex), 
    .aluSrc_idex(aluSrc_idex),
    .memToReg_idex(memToReg_idex)
);

registerFile regis(
    .clk(clk),
    .writeEnable(regWrite_memwb),
    .destAdd_wb(dest_addr_memwb),
    .destAdd_dataFetch(destAddrReg_idex),
    .r1_add(operand_1_idex),
    .r2_add(operand_2_idex), //returns xxxx if a number larger than 32 is passed
    .writeData(dataToWrite),

    .r1(r1_reg),
    .r2(r2_reg),
    .destAddVal(destAddVal_reg)
);


aluSrcMuxer muxe(
    .aluSrc(aluSrc_idex),
    .r2_register(r2_reg),
    .intermediate(operand_2_idex),

    .aluInput2(aluInput2)

);

ALU exec(
    .clk(clk),
    .aluTask(aluTask_idex),
    .operand1(r1_reg),
    .operand2(aluInput2),
    .shamt(shift_idex),
    .result(alu_result)
);

exmem_register emReg(
    .clk(clk),
    .alu_result(alu_result), 
    .r2(r2_reg),
    .destAddVal(destAddVal_reg),
    .dest_addr(destAddrReg_idex), // for writing back into a register
    .regWrite(regWrite_idex),
    .memRead(memRead_idex),
    .memWrite(memWrite_idex),
    .memToReg(memToReg_idex),

    .alu_result_exmem(alu_result_exmem),
    .r2_exmem(r2_exmem),
    .destAddVal_exmem(destAddVal_exmem),
    .dest_addr_exmem(dest_addr_exmem),
    .regWrite_exmem(regWrite_exmem),
    .memRead_exmem(memRead_exmem),
    .memWrite_exmem(memWrite_exmem),
    .memToReg_exmem(memToReg_exmem)
);


storage memBlock(
    .clk(clk),
    .addr(alu_result_exmem),
    .data_toMemory(destAddVal_exmem),
    .memRead(memRead_exmem),
    .memWrite(memWrite_exmem),
    .data_fromMemory(data_fromMemory)
);

memwb_register mwReg(
    .clk(clk),
    .mem_data(data_fromMemory),
    .alu_result(alu_result_exmem),
    .dest_addr(dest_addr_exmem),
    .regWrite(regWrite_exmem),
    .memToReg(memToReg_exmem),

    .mem_data_memwb(mem_data_memwb),
    .alu_result_memwb(alu_result_memwb),
    .dest_addr_memwb(dest_addr_memwb),
    .regWrite_memwb(regWrite_memwb),
    .memToReg_memwb(memToReg_memwb)
);

wbDataMux wbDataSelector(
    .memToReg_memwb(memToReg_memwb),
    .mem_data_memwb(data_fromMemory),
    .alu_result_memwb(alu_result_memwb),

    .dataToWrite(dataToWrite)
);


always @(posedge clk) begin
pc <= pc + 1;
end

/*
At clock 0 (first positive edge), an instruction is fetched, and the register module STARTS storing it
    At the same time, the decode module STARTS reading what's in the register module, i.e., nothing
    At the same time, the execute module (ALU) STARTS reading what's in the register module between it and the decode module, i.e., nothing

BUT! At clock 1 (second positive edge), the registers have some value, and the next module will read what's in the register

The cycle continues
*/

endmodule