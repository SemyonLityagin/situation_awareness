#include "IdGenerator.h"

uint32_t IdGenerator::idCounter = 0;

uint32_t IdGenerator::get_id(){
    return this->idCounter++;
}