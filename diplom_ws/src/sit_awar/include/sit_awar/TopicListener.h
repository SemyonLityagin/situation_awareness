#pragma once

#include "SitAwarMsgsTypedef.h"
#include "json.hpp"

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class TopicListener {
    std::vector<SensorPack::SharedPtr> sensorsSequence;
    SelfObjectState::SharedPtr selfState;

    rclcpp::Node::SharedPtr node;
    std::vector<rclcpp::Subscription<SensorPack>::SharedPtr> subcribers;
    rclcpp::Subscription<SelfObjectState>::SharedPtr selfStatesSubcriber;

public:
    TopicListener(rclcpp::Node::SharedPtr node);
    std::vector<SensorPack::SharedPtr> pop_sensors();
    SelfObjectState::SharedPtr pop_self_state();
    void spin_some();
    void subscriber_pack_callback(const SensorPack::SharedPtr msg);
    void subscriber_self_state_callback(const SelfObjectState::SharedPtr msg);
};