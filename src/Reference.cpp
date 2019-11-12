#include "Reference.hpp"

Reference::Reference() : Block(block_type::reference){
    if(_type == block_type::reference)
        std::cout << "This is a Reference block" << std::endl; 
    else
        std::cout << "NOT WHATEVER" << std::endl;
    //while(1);   
    _status = false; 
}

Reference::~Reference() {

}

block_type Reference::getType() {
    return _type;
}

void Reference::switchIn(){
    _status = true;

}

void Reference::switchOut(){
    _status = false;

}   

void Reference::receive_msg_data(uint8_t data[],std::size_t len, msg_type _msg_type){


}