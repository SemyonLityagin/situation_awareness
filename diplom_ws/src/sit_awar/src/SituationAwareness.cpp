#include "SituationAwareness.h"

SituationAwareness::SituationAwareness(
                            std::shared_ptr<WorldModel> worldModel,
                            std::shared_ptr<DataAggregation> dataAggregation,
                            std::shared_ptr<DataFilter> dataFilter,
                            rclcpp::Node::SharedPtr node
                            ) {
    this->topicListener = std::make_shared<TopicListener>(node);
    this->worldModel = worldModel;
    this->idGenerator = std::make_shared<IdGenerator>(IdGenerator());
    this->dataFilter = dataFilter;
    this->dataAggregation = dataAggregation;
    this->currentTime = 0;
};

// cycle of SitAwar:
// get SensorPacks
// aggregate
// filtering
// update model
// update time
void SituationAwareness::timer_callback() {
    std::vector<SensorPack::SharedPtr> sensorSequence = this->topicListener->pop_sensors();
    SelfObjectState::SharedPtr selfState = this->topicListener->pop_self_state();
    this->worldModel->update_time(this->currentTime);
    this->worldModel->update_self_object(selfState);

    std::vector<SensorData> aggregatedObjects = this->dataAggregation->do_aggregation(sensorSequence);
    std::cout << "From agg get " << aggregatedObjects.size() << " obj\n";
    std::unordered_map<uint32_t, ObjectState> filteredObjects = this->dataFilter->get_filtered_objects(aggregatedObjects, idGenerator);
    std::cout << "From filter get " << filteredObjects.size() << " obj\n";
    this->worldModel->update_model(filteredObjects);
    this->currentTime++;
    std::cout << this->currentTime << std::endl;

};
