#pragma once

#include "SitAwarMsgsTypedef.h"

class DataAggregation {
public:
    virtual std::vector<SensorData> do_aggregation(std::vector<SensorPack::SharedPtr>& sensorsSequence)=0;
};