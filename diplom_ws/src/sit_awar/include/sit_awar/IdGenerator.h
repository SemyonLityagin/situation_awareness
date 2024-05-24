#pragma once

#include <cstdint>

class IdGenerator {
private:
    static uint32_t idCounter;
public:
    uint32_t get_id();
};