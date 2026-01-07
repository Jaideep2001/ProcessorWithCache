//FETCH -> if_de_Register -> DECODE -> de_ex_Reguster -> EXECUTE -> ex_mem_Register -> MEMORY -> mem_wb_Register -> WRITE BACK

module instructionMemory(
    input clk,
    input [5:0] address,
    input stallForMemoryAccess,
    output reg [31:0] instruction
);

    reg [31:0] memory[0:14]; // Internal ROM

    initial begin
        $readmemh("program_cacheTest.mem", memory);
    end
    
    always @(*) begin
        instruction = memory[address];
    end
endmodule


module registerFile(
    input clk,
    input writeEnable,
    input stallForMemoryAccess,
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
    if (!(stallForMemoryAccess)) begin
        if (writeEnable && destAdd_wb != 0) begin
            registers[destAdd_wb] <= writeData;
        end
    end
    $writememh("reg_dump.mem", registers);
end

endmodule

module storage(
    input clk,
    input [7:0] addr,
    input [7:0] addr_fromCache,
    input [31:0] data_toMemory,
    input memRead, memWrite,
    output reg [31:0] data_fromMemory,
    input hitOrMiss,
    output reg memDataCorrect
);

reg [7:0] DataMemory [0:255];
integer i;
reg clkDiv = 0;
reg [31:0] temp_data;
reg writeInProgress = 0;
reg readDone = 0;
reg [1:0] writeCounter = 0;

initial begin
    $readmemh("memoryData.mem", DataMemory);
end
/*
always @(*) begin
    memDataCorrect = hitOrMiss;
end
*/
always @(posedge clk) begin
    //memDataCorrect has to be set to 1 only when data gets written.
    memDataCorrect = hitOrMiss;
    if (memRead) begin
        memDataCorrect = 0;
        clkDiv <= clkDiv + 1; //intentional delay of 1 cycle
        if (clkDiv) begin
            temp_data = 32'd0;
            if (memWrite == 0) begin
                memDataCorrect <= 1;
            end
            for (i = 0; i < 4; i = i + 1) begin
                temp_data = (temp_data << 8) | DataMemory[(addr & 8'b11111100) + i];
            end
            data_fromMemory <= temp_data;
            clkDiv <= 0;
            readDone <= 1;
        end
    end

    if (memWrite) begin
        if (writeInProgress == 0) begin
            memDataCorrect = 0;
            writeInProgress = 1;
            writeCounter = 0;
            // 1 cycle delay + write initiation (I guess?)
        end

        if (writeInProgress == 1) begin
            DataMemory[(addr_fromCache & 8'b11111100) + writeCounter] <= (data_toMemory >> (4-writeCounter-1)*8);
            writeCounter <= writeCounter + 1;
        end

        if (writeCounter == 2'b11) begin
            memDataCorrect <= 1;
            writeInProgress <= 0;
            writeCounter <= 0;
        end
    end
    $writememh("memory_dump.mem", DataMemory);
end
endmodule

module cacheMemory(
    input clk,
    input [7:0] address,
    input [7:0] data_toCache,
    input [31:0] data_fromMemory,
    input memDataCorrect,
    input writeCache,
    input readCache,
    output reg [7:0] addr_fromCache,
    output reg hitOrMiss,
    output reg memRead,
    output reg memWrite,
    output reg [31:0] data_toMemory,
    output reg [7:0] data_toProcessor
);

// BLOCK: VALID BIT (1) | TAG (4) | DIRTY BIT (1) | DATA (32)

reg [3:0] tag;
reg [1:0] index;
reg [1:0] offset;
reg [4:0] whatsInCache;
reg [37:0] cache [0:3];

integer i;

initial begin
    memRead = 0;
    memWrite = 0;
    hitOrMiss = 1;           // Assume hit until first logic runs
    data_toMemory = 32'd0;
    data_toProcessor = 8'd0;

    // Initialize cache to zero to avoid undefined behavior
    for (i = 0; i < 4; i = i + 1) begin
        cache[i] = 38'd0;
    end
    i = 0;
end


always @(*) begin
    index = (address >> 6);
    tag = (address >> 2) & 4'b1111;
    offset = address & 2'b11;
    whatsInCache = (cache[index] >> 33); 
    if ((whatsInCache) == {1'b1, tag}) begin
        hitOrMiss = 1;
    end

    else if (readCache || writeCache) begin
        hitOrMiss = 0;
    end
end


always @(posedge clk) begin
    //address = 0x00001011

    // dirty bit = 1

    if ((cache[index] >> 33) == {1'b1, tag}) begin
        //Cache hit lesgo
        if (writeCache == 1) begin
            memWrite <= 0;
            memRead <= 0;
            //hitOrMiss = 1;
            cache[index] <= ((cache[index] & (~((38'b11111111) << 8*(4-offset-1)))) | ({30'd0, data_toCache} << 8*(4-offset-1))) | (38'b1 << 32); //setting Dirty Bit to 1
            //cache[index] <= cache[index] | (38'b1 << 32); //setting Dirty Bit to 1
        end

        else if (readCache == 1) begin
            memWrite <= 0;
            memRead <= 0;
            //hitOrMiss = 1;
            data_toProcessor <= (cache[index] >> ((4-offset-1)*8)) & 8'b11111111;
        end
    end

    //Cache miss: gotta read

    else if ((cache[index] >> 32 ) & 1'b1) begin
        //Dirty data has to be written to main memory
        if (readCache || writeCache) begin
            //hitOrMiss = 0;
            data_toMemory <= (cache[index] & 32'hFFFFFFFF);
            addr_fromCache <= {index, ((cache[index] >> 33) & 4'b1111 ), 2'b00};
            memWrite <= 1;
            memRead <= 1;
            if (memDataCorrect == 1) begin
                cache[index] <= {1'b1, tag, 1'b0 /* dirty set to zero */, data_fromMemory};
            end
        end
    end
    else begin
        if (readCache || writeCache) begin
            //hitOrMiss = 0;
            memWrite <= 0;
            memRead <= 1;
            if (memDataCorrect == 1) begin
                cache[index] = {1'b1, tag, 1'b0 /* dirty set to zero */, data_fromMemory};
            end
        end
    end

    $writememh("cache_dump.mem", cache);
end
endmodule

module decode(
    input clk,
    input [31:0] instruction,
    output reg [3:0] aluTask,
    output reg [4:0] operand_1, destAddrReg, shift,
    output reg [15:0] operand_2, jumpAddress,
    output reg regWrite, memRead, memWrite, aluSrc, memToReg //flags
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
LW          Load Word               4'b1001
SW          Store Word              4'b1010
BEQ         Branch If Equals        4'b1011
BNE         Branch Not Equals       4'b1100
*/

//R type instruction: Opcode (6bits), Reg1 (5 bits), Reg2 (5 bits), DestReg (5 bits), Shift (5 bits), Function (6bits)
//I type instruction: Opcode (6bits), Reg1 (5 bits), Reg2 (5 bits), Immediate (16 bits)
//J type instruction (branching. Not jumping yet): Opcode (6bits), Reg1 (5 bits), Reg2 (5 bits), Jump Target (16 bits)

reg [5:0] opcode, funct;

always @(*) begin
    opcode = (instruction >> 26);

    case((instruction >> 26))
        6'b000000: begin
            regWrite = 1'b1;
            memRead = 1'b0;
            memWrite = 1'b0;
            memToReg = 1'b0;
            aluSrc = 1'b0;

            //ADD R1, R2, R3: 000000 00010 00011 00001 00000 000001

            funct = (instruction & 6'b111111);
            aluTask = funct[3:0];
            operand_1 = ((instruction >> 21) & 5'b11111);
            operand_2 = ((instruction >> 16) & 5'b11111);
            destAddrReg = ((instruction >> 11) & 5'b11111);
            shift = ((instruction >> 6) & 5'b11111);

        end
        6'b000001, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110, 6'b000111, 6'b001000: begin
            regWrite = 1'b1;
            memRead = 1'b0;
            memWrite = 1'b0;
            memToReg = 1'b0;
            aluSrc = 1'b1;

            aluTask = instruction[29:26];

            //aluTask = (instruction >> 26)[3:0];

            //ADDI R1, R2, 50: 000001 00010 00001 0000000000110010

            destAddrReg = ((instruction >> 16) & 5'b11111);
            operand_1 = ((instruction >> 21) & 5'b11111);
            operand_2 = (instruction & 16'hFFFF);
        end
        6'b001001: begin //load word
            regWrite = 1'b1;
            memRead = 1'b1;
            memWrite = 1'b0;
            memToReg = 1'b1;
            aluSrc = 1'b1;

            aluTask = 4'b0001;

            //LW R1, 100(R2): 001001 (op), 00010 (R2), 00001 (R1), 0000000001100100 (#100)
            destAddrReg = ((instruction >> 16) & 5'b11111);
            operand_1 = ((instruction >> 21) & 5'b11111);
            operand_2 = (instruction & 16'hFFFF);
        end
        6'b001010: begin //store word
            memRead = 1'b0;
            memWrite = 1'b1;
            regWrite = 1'b0;
            memToReg = 1'b0;
            aluSrc = 1'b1;

            aluTask = 4'b0001;

            //SW R1, 100(R2): 001010 (op), 00010 (R2), 00001 (R1), 0000000001100100 (#100)

            destAddrReg = ((instruction >> 16) & 5'b11111);
            operand_1 = ((instruction >> 21) & 5'b11111);
            operand_2 = (instruction & 16'hFFFF);
        end
        6'b001011, 6'b001100: begin //branch
            aluTask = instruction[29:26];
            aluSrc = 1'b0;
            operand_1 = ((instruction >> 21) & 5'b11111);
            operand_2 = ((instruction >> 16) & 5'b11111);
            jumpAddress = (instruction & 16'hFFFF);
        end

        default : begin
        regWrite = 0;
        memRead = 0;
        memWrite = 0;
        memToReg = 0;
        aluSrc = 0;
        aluTask = 4'b0000;
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
    output reg [7:0] result,
    output reg jumpFlag
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
BEQ         Branch If Equals        4'b1011
BNE         Branch Not Equals       4'b1100
*/

always @(*) begin
    jumpFlag = 0;
    case (aluTask)
        4'b0001: begin
            result = operand1 + operand2[7:0];
        end

        4'b0010: begin
            result = operand1 - operand2[7:0];
        end

        4'b0011: begin
            result = operand1 & operand2[7:0];
        end

        4'b0100: begin
            result = operand1 | operand2[7:0];
        end

        4'b0101: begin
            result = operand1 ^ operand2[7:0];
        end

        4'b0110: begin
            result = operand1[7:0] << shamt;
        end

        4'b0111: begin
            result = operand1[7:0] >> shamt;
        end

        4'b1000: begin
            result = (operand1 < operand2) ? 1 : 0;
        end 

        4'b1011: begin
            result = (operand1 == operand2) ? 1 : 0;
            if (result == 1) begin
                jumpFlag = 1;
            end
        end

        4'b1100: begin
            result = (operand1 != operand2) ? 1 : 0;
            if (result == 1) begin
                jumpFlag = 1;
            end
        end

        default: result <= 8'b0;
    endcase
end
endmodule

//IF/ID: Holds fetched instruction + PC.

module ifid_register(
    input clk,
    input [5:0] programCounter,
    input stallForMemoryAccess,
    input [31:0] instruction,
    output reg [31:0] instr_ifid,
    output reg [5:0] pc_ifid
);

always @(posedge clk) begin
    if (!(stallForMemoryAccess)) begin
        instr_ifid <= instruction;
        pc_ifid <= programCounter;
    end
end
endmodule

//ID/EX: Holds decoded signals + register values.

module idex_register(
    input clear,
    input clk,
    input stallForMemoryAccess,
    input [3:0] aluTask,
    input [4:0] operand_1, destAddrReg, shift,
    input [15:0] operand_2, jumpAddress,
    input regWrite, memRead, memWrite, aluSrc, memToReg,
    input [5:0] pc_ifid,

    output reg [3:0] aluTask_idex,
    output reg [4:0] operand_1_idex, destAddrReg_idex, shift_idex,
    output reg [15:0] operand_2_idex, jumpAddress_idex,
    output reg regWrite_idex, memRead_idex, memWrite_idex, aluSrc_idex, memToReg_idex, //flags
    output reg [5:0] pc_idex
);

always @(posedge clk) begin
    if (!(stallForMemoryAccess)) begin
        aluTask_idex <= aluTask;
        operand_1_idex <= operand_1;
        destAddrReg_idex <= destAddrReg;
        shift_idex <= shift;
        operand_2_idex <= operand_2;
        jumpAddress_idex <= jumpAddress;
        regWrite_idex <= regWrite;
        memRead_idex <= memRead;
        memWrite_idex <= memWrite;
        aluSrc_idex <= aluSrc;
        memToReg_idex <= memToReg;
        pc_idex <= pc_ifid;
    end
end
endmodule

//EX/MEM: Holds ALU result + memory control.

module exmem_register (
    input clk,
    input stallForMemoryAccess,
    input [7:0] alu_result, r2, destAddVal,
    input [4:0] dest_addr, // for writing back into a register
    input regWrite, memRead, memWrite, memToReg,

    output reg [7:0] alu_result_exmem, r2_exmem, destAddVal_exmem,
    output reg [4:0] dest_addr_exmem,
    output reg regWrite_exmem, memRead_exmem, memWrite_exmem, memToReg_exmem
);

always @(posedge clk) begin
    if (!(stallForMemoryAccess)) begin
        alu_result_exmem <= alu_result;
        r2_exmem <= r2;
        destAddVal_exmem <= destAddVal;
        dest_addr_exmem <= dest_addr;
        regWrite_exmem <= regWrite;
        memRead_exmem <= memRead;
        memWrite_exmem <= memWrite;
        memToReg_exmem <= memToReg;
    end
end
endmodule

//MEM/WB: Holds memory read data or ALU result.

module memwb_register(
    input clk,
    input stallForMemoryAccess,
    input [7:0] mem_data, alu_result,
    input [4:0] dest_addr,
    input regWrite, memToReg,

    output reg [7:0] mem_data_memwb, alu_result_memwb,
    output reg [4:0] dest_addr_memwb,
    output reg regWrite_memwb, memToReg_memwb
);

always @(posedge clk) begin
    if (!(stallForMemoryAccess)) begin
        mem_data_memwb <= mem_data;
        alu_result_memwb <= alu_result;
        dest_addr_memwb <= dest_addr;
        regWrite_memwb <= regWrite;
        memToReg_memwb <= memToReg;
    end
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
wire [5:0] pc_ifid;

wire [3:0] aluTask;
wire [4:0] operand_1, destAddrReg, shift;
wire [15:0] operand_2, jumpAddress;
wire regWrite, memRead, memWrite, aluSrc, memToReg; //flags

wire [3:0] aluTask_idex;
wire [4:0] operand_1_idex, destAddrReg_idex, shift_idex;
wire [15:0] operand_2_idex, jumpAddress_idex;
wire regWrite_idex, memRead_idex, memWrite_idex, aluSrc_idex, memToReg_idex; //flags
wire [5:0] pc_idex;

wire [7:0] r1_reg, r2_reg, destAddVal_reg;

wire [15:0] aluInput2;

wire [7:0] alu_result;
wire jumpFlag;

wire [7:0] alu_result_exmem, r2_exmem, destAddVal_exmem;
wire [4:0] dest_addr_exmem;
wire regWrite_exmem, readCache, writeCache, memToReg_exmem;

// Storage & Cache block

wire stallForMemoryAccess;

wire hitOrMiss;

wire [31:0] data_fromMemory;
wire memDataCorrect;
wire memRead_fromCache;
wire memWrite_fromCache;
wire [7:0] addr_fromCache;
wire [31:0] data_toMemory;
wire [7:0] data_toProcessor;

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
    .programCounter(pc),
    .stallForMemoryAccess(stallForMemoryAccess),
    .instruction(instruction),
    .instr_ifid(instr_ifid),
    .pc_ifid(pc_ifid)
);

decode dec(
    .clk(clk),
    .instruction(instr_ifid),
    .aluTask(aluTask),
    .operand_1(operand_1),
    .destAddrReg(destAddrReg),
    .shift(shift),
    .operand_2(operand_2),
    .jumpAddress(jumpAddress_idex),
    .regWrite(regWrite),
    .memRead(memRead),
    .memWrite(memWrite),
    .aluSrc(aluSrc),
    .memToReg(memToReg)
);

idex_register ieReg(
    .clk(clk),
    .stallForMemoryAccess(stallForMemoryAccess),
    .aluTask(aluTask),
    .operand_1(operand_1),
    .destAddrReg(destAddrReg),
    .shift(shift),
    .operand_2(operand_2),
    .jumpAddress(jumpAddress),
    .regWrite(regWrite),
    .memRead(memRead),
    .memWrite(memWrite),
    .aluSrc(aluSrc),
    .memToReg(memToReg),
    .pc_ifid(pc_ifid),

    .aluTask_idex(aluTask_idex),
    .operand_1_idex(operand_1_idex), 
    .destAddrReg_idex(destAddrReg_idex), 
    .shift_idex(shift_idex),
    .operand_2_idex(operand_2_idex),
    .jumpAddress_idex(jumpAddress),
    .regWrite_idex(regWrite_idex), 
    .memRead_idex(memRead_idex), 
    .memWrite_idex(memWrite_idex), 
    .aluSrc_idex(aluSrc_idex),
    .memToReg_idex(memToReg_idex),
    .pc_idex(pc_idex)
);

registerFile regis(
    .clk(clk),
    .stallForMemoryAccess(stallForMemoryAccess),
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
    .result(alu_result),
    .jumpFlag(jumpFlag)
);

exmem_register emReg(
    .clk(clk),
    .stallForMemoryAccess(stallForMemoryAccess),
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
    .memRead_exmem(readCache),
    .memWrite_exmem(writeCache),
    .memToReg_exmem(memToReg_exmem)
);

cacheMemory cacheBlock(
    .clk(clk),
    .address(alu_result_exmem),
    .data_toCache(destAddVal_exmem),
    .data_fromMemory(data_fromMemory),
    .memDataCorrect(memDataCorrect),
    .addr_fromCache(addr_fromCache),
    .writeCache(writeCache),
    .readCache(readCache),
    .hitOrMiss(hitOrMiss),
    .memRead(memRead_fromCache),
    .memWrite(memWrite_fromCache),
    .data_toMemory(data_toMemory),
    .data_toProcessor(data_toProcessor)
);

storage memBlock(
    .clk(clk),
    .addr(alu_result_exmem),
    .addr_fromCache(addr_fromCache),
    .data_toMemory(data_toMemory),
    .memRead(memRead_fromCache),
    .memWrite(memWrite_fromCache),
    .data_fromMemory(data_fromMemory),
    .hitOrMiss(hitOrMiss),
    .memDataCorrect(memDataCorrect)
);

memwb_register mwReg(
    .clk(clk),
    .stallForMemoryAccess(stallForMemoryAccess),
    .mem_data(data_toProcessor),
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
    .mem_data_memwb(data_toProcessor),
    .alu_result_memwb(alu_result_memwb),

    .dataToWrite(dataToWrite)
);

//CONTROL LOGIC

assign stallForMemoryAccess = ~(hitOrMiss);

always @(posedge clk) begin
if (hitOrMiss != 0) begin
    if (jumpFlag == 1) begin
        pc <= pc_idex + 1 + jumpAddress_idex[5:0];
    end
    else
        pc <= pc + 1;
end

end

/*
At clock 0 (first positive edge), an instruction is fetched, and the register module STARTS storing it
    At the same time, the decode module STARTS reading what's in the register module, i.e., nothing
    At the same time, the execute module (ALU) STARTS reading what's in the register module between it and the decode module, i.e., nothing

BUT! At clock 1 (second positive edge), the registers have some value, and the next module will read what's in the register

The cycle continues
*/

endmodule