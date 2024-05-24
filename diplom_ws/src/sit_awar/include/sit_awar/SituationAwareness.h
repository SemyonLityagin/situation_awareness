#pragma once

#include "WorldModel.h"
#include "DataFilter.h"
#include "DataAggregation.h"
#include "IdGenerator.h"
#include "TopicListener.h"
#include <rclcpp/rclcpp.hpp>

class SituationAwareness {
    std::shared_ptr<WorldModel> worldModel;
    std::shared_ptr<IdGenerator> idGenerator;
    std::shared_ptr<DataFilter> dataFilter;
    std::shared_ptr<DataAggregation> dataAggregation;
    std::shared_ptr<TopicListener> topicListener;
    int64_t currentTime;

public:
    SituationAwareness(
        std::shared_ptr<WorldModel> worldModel,
        std::shared_ptr<DataAggregation> dataAggregation,
        std::shared_ptr<DataFilter> dataFilter,
        rclcpp::Node::SharedPtr node
        );
    void timer_callback();
};