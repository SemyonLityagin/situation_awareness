#pragma once

#include "SitAwarMsgsTypedef.h"
#include "Object.h"

#include <map>

class WorldModel {
    std::shared_ptr<std::unordered_map<uint32_t, Object>> friendObjects;
    std::shared_ptr<std::unordered_map<uint32_t, Object>>  enemyObjects;
    SelfObjectState::SharedPtr selfObject;
    int64_t currentTime;

public:
    WorldModel();
    void update_time(int64_t time);
    void update_model(std::unordered_map<uint32_t, ObjectState>& filteredObjects);
    void update_self_object(SelfObjectState::SharedPtr selfObject);
    std::shared_ptr<std::unordered_map<uint32_t, Object>> get_history(bool friendly);
};