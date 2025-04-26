#ifndef __IOSTREAM_H
#include <iostream>
#endif

#ifndef __VECTOR_H
#include <vector>
#endif


class Memory_block{
    public:
    int tag, valid_bit, dirty_bit, replacement_info;
    std::vector<int> data;
    Memory_block(int address_tag, int valid, int dirty, int replacement, std::vector<int> data_store)
        :tag(address_tag), valid_bit(valid), dirty_bit(dirty), replacement_info(replacement), data(data_store){}  
};