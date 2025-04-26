#include <iostream>
#include <vector>
#include "memBlock.hpp"

std::vector<std::vector<int>> main_memory(16, std::vector<int>(16,5));
std::vector<Memory_block> cache;
//4 blocks, 16 byte block length.

void init_cache(){
    int i = 0;
    while (i<4){
        cache.push_back(Memory_block(0,0,0,-1,std::vector<int>(16,0)));
        i++;
    }
}

int read(int address){
    //First, we must go to cache to check if it has what we need
    int offset_add = address & 0b1111;
    int index_add = (address & 0b110000) >> 4;
    int tag_add = address & 0b11000000;
    int read_from_cache_flag = 1;

    //if yes, we fetch it from cache

    for (int i = 0; i<cache.size(); i++){
        if (i == index_add){
            if (cache[i].valid_bit == 0){
                read_from_cache_flag = 0;
            }
            else if (cache[i].tag == tag_add){
                return cache[i].data[offset_add];
            }
            else{
                read_from_cache_flag = 0;
            }
        }
        if (read_from_cache_flag == 0){break;}
    }

    //if not, we go to memory, fetch it, and write it in cache
    int row = (address & 0b11110000) >> 4;

    if (cache[index_add].dirty_bit == 1){
        int row = (cache[index_add].tag | (index_add << 4)) >> 4;
        for (int i = 0; i < cache[index_add].data.size(); i++){
            main_memory[row][i] = cache[index_add].data[i];
        }
        cache[index_add].dirty_bit = 0;
    }

    cache[index_add].tag = tag_add;
    for (int i = 0; i< cache[index_add].data.size(); i++){
        cache[index_add].data[i] = main_memory[row][i];
    }
    cache[index_add].valid_bit = 1;
    return cache[index_add].data[offset_add];

}

void write(int address, int data, int length = 1 /*add the unaligned write functionality later*/){

    //if found in cache, we write in cache
    int offset_add = address & 0b1111;
    int index_add = (address & 0b110000) >> 4;
    int tag_add = address & 0b11000000;
    int write_in_cache_flag = 1;

    for (int i = 0; i<cache.size(); i++){
        if (i == index_add){
            if (cache[i].valid_bit == 0){
                write_in_cache_flag = 0;
            }
            else if (cache[i].tag == tag_add){
                cache[i].data[offset_add] = data;
                cache[i].dirty_bit = 1;
            }
            else{
                write_in_cache_flag = 0;
            }
        }
        if (write_in_cache_flag == 0){break;}
    }

    //if not found, we write in memory
    int row = (address & 0b11110000) >> 4;
    main_memory[row][offset_add] = data;

}

int main(){
    //main program
    init_cache();
    int address = 0b10110110;
    int value = read(address);
    std::cout<<value<<std::endl;
    value = read(address);
    std::cout<<value<<std::endl;
    write(address, 70);
    value = read(address);
    std::cout<<value<<std::endl;
    address = 0b01110110;
    value = read(address);
    std::cout<<value<<std::endl;
    return 0;
}