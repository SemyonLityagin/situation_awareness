#pragma once

#include "SitAwarMsgsTypedef.h"
// #include "WorldModel.h"
#include "IdGenerator.h"

#include <map>
#include <cstdint>

class DataFilter {
public:
    virtual std::unordered_map<uint32_t, ObjectState> get_filtered_objects(
        std::vector<SensorData> &aggregatedObjects,
        // std::shared_ptr<WorldModel> worldModel,
        std::shared_ptr<IdGenerator> IdGenerator 
        )=0;
};

