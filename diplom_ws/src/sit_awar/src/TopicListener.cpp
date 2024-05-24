#include "TopicListener.h"

const std::string LOCAL_MODULE = "local";
const std::string CONFIG_DIR = "/agent_config/agent_config.json";

TopicListener::TopicListener(rclcpp::Node::SharedPtr node) {
    // load config file
    this->node = node;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("sit_awar");

    std::ifstream f(package_share_directory + CONFIG_DIR);
    nlohmann::json json_data = nlohmann::json::parse(f);
    std::string agent_name = json_data["agent_name"];

    this->sensorsSequence = std::vector<SensorPack::SharedPtr>();
    this->selfState = nullptr;
    
    // create subscribers
    for (auto& el : json_data["sensors"].items()) {
        nlohmann::json val = el.value();
        if(LOCAL_MODULE == val["sensor"]) {
            this->selfStatesSubcriber = this->node->create_subscription<SelfObjectState>(val["topic"], rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&TopicListener::subscriber_self_state_callback, this, std::placeholders::_1));
        } else {
            subcribers.push_back(
                this->node->create_subscription<SensorPack>(val["topic"], rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&TopicListener::subscriber_pack_callback, this, std::placeholders::_1))
            );
        }
    }
    
};

std::vector<SensorPack::SharedPtr> TopicListener::pop_sensors() {
    std::vector<SensorPack::SharedPtr> result = this->sensorsSequence;
    this->sensorsSequence = std::vector<SensorPack::SharedPtr>();
    return result;
};

SelfObjectState::SharedPtr TopicListener::pop_self_state() {
    return this->selfState;
};

void TopicListener::subscriber_pack_callback(const SensorPack::SharedPtr msg) {
    std::cout << "TL get new Pack with " << msg->objects.size() << " obj" << "\n";
    this->sensorsSequence.push_back(msg);
};

void TopicListener::subscriber_self_state_callback(const SelfObjectState::SharedPtr msg) {
    std::cout << "TL get new state\n";
    this->selfState = msg;
};