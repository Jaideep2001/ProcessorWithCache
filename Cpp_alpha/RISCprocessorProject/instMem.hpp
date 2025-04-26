#ifndef __IOSTREAM_H
#include <iostream>
#endif

#ifndef __VECTOR_H
#include <vector>
#endif

class Instruction{
    private:
        std::string opcode;
        int rd, r1, r2;
    public:
        //for instructions with 2 operands
        Instruction(std::string op, int d, int o1, int o2):opcode(op),rd(d),r1(o1),r2(o2) {}

        //for instructions with one operand
        Instruction(std::string op, int d, int o1): opcode(op),rd(d),r1(o1) {}
    
        std::string get_opcode(){return opcode;}
        int get_destination(){return rd;}
        int get_op1(){return r1;}
        int get_op2(){return r2;}
        
};