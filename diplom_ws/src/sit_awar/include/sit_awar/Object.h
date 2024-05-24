#include <cstdint>

#include "consts.h"
#include "SitAwarMsgsTypedef.h"
#include <list>

class Object{
public:
    uint32_t id;
    std::list<ObjectState> states;

    Object(uint32_t id, ObjectState state){
        this->id = id;
        states = std::list<ObjectState>();
        states.push_back(state);
    }
    void update_state(ObjectState state){
        states.push_front(state);
        if(states.size() > HISTORY_DEEP){
            states.pop_back();
        }
    };
};