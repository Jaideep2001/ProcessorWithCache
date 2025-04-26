#include <iostream>
#include <vector>
#include "instMem.hpp"
#include <map>

std::map<std::string, int> opcode_lookup = {
    { "ADD", 0 },
    { "LDR", 1 },
    { "STR", 2 },
    { "BEZ", 3 },
    { "BNEZ", 4},
    { "BEQ", 5},
    { "BNE", 6},
    { "J", 7}
};

int registers[32] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,};
std::vector<int> storagememory(64,0);

struct decode_res {public: int destin_reg; int opcode; int operand1; int operand2;};

class InstructionMemory{
    private:
        std::vector<Instruction> memory;
    public:
    InstructionMemory() {
        memory.push_back(Instruction("ADD", 1, 2, 3));    // ADD R1, R2, R3
        memory.push_back(Instruction("ADD", 2, 3, 4));    // ADD R2, R3, R4
        memory.push_back(Instruction("STR", 1, 5, 3));    // STR mem[R5+R3], R1
        memory.push_back(Instruction("LDR", 2, 8, 0));    // LDR R1, [R2]
    }

    int memSize(){
        return memory.size();
    }

    Instruction retrieve_inst(int addr){
        return memory[addr];
    }
    
}memory1;

Instruction fetch(InstructionMemory memory, int addr){
    if ((addr>memory.memSize()) || (addr<0)){
        throw std::runtime_error("Attempting to access non-existent memory.");
    }
    return memory.retrieve_inst(addr);
}

auto decode(Instruction instr, std::map<std::string, int> opcode_lookup_loc = opcode_lookup){
    auto it = opcode_lookup_loc.find(instr.get_opcode());
    if (it == opcode_lookup_loc.end()){
        throw std::runtime_error("Opcode does not exist.");
    }
    int opc = it->second;
    int dest_reg, op1, op2;
    dest_reg = instr.get_destination();
    if ((opc == 0) || (opc == 1) || (opc == 2)){
        op1 = registers[instr.get_op1()];
        op2 = registers[instr.get_op2()];
    }
    return decode_res{dest_reg, opc, op1, op2};
}

int alu(int opc, int op1, int op2);

int execute(decode_res decode_result){
    int opc = decode_result.opcode, result;
    result = alu(opc, decode_result.operand1, decode_result.operand2);
    //std::cout<<decode_result.opcode<<" "<<decode_result.destin_reg<<" "<<decode_result.operand1<<" "<<decode_result.operand2<<" "<<result<<std::endl;
    return result;
}

void memAccess(int opcode, int destReg, int secondOp){

    if (opcode == 1){
        registers[destReg] = storagememory[secondOp];
    }
    if (opcode == 2){
        storagememory[secondOp] = registers[destReg];
    }
}

void writeBack(int opcode, int result, int destReg){
    if (opcode == 2){return;}
    registers[destReg] = result;
}

int main(){
    int pc = 0;
    decode_res deconding_result;
    while (pc<memory1.memSize()){
        Instruction temp_instr = fetch(memory1,pc);
        deconding_result = decode(temp_instr);
        int result = execute(deconding_result);
        memAccess(deconding_result.opcode, deconding_result.destin_reg, result);
        writeBack(deconding_result.opcode, result, deconding_result.destin_reg);
        std::cout<<"Value at reg1: "<<registers[1]<<std::endl<<"Value at reg2: "<<registers[2]<<std::endl;
        std::cout<<std::endl;
        pc+=1;
    }
    return 0;
}

int alu(int opc, int op1, int op2){
    int result;
    if ((opc==0) || (opc == 1) || (opc == 2)){result = op1+op2;}
    return result;
}