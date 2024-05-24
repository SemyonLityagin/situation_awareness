#include "WorldModel.h"

#include <rclcpp/rclcpp.hpp>

WorldModel::WorldModel() {
    this->currentTime = 0;
    this->enemyObjects = std::make_shared<std::unordered_map<uint32_t, Object>>();
    this->friendObjects = std::make_shared<std::unordered_map<uint32_t, Object>>();
    this->selfObject = SelfObjectState::SharedPtr();
}
void WorldModel::update_time(int64_t time) {
    this->currentTime = time;
}

void WorldModel::update_model(std::unordered_map<uint32_t, ObjectState> &filteredObjects) {
    for (auto obj = filteredObjects.begin(); obj != filteredObjects.end(); obj++){
        bool friendly = obj->second.obj_sensor_data.friendly;
        
        auto objects = friendly ? this->friendObjects : this->enemyObjects;
        auto f_obj = objects->find(obj->first);
        if (f_obj == objects->end()){
            objects->insert({obj->first, Object(obj->first, obj->second)});
        } else {
            f_obj->second.update_state(obj->second);
        }
    }
}

void WorldModel::update_self_object(SelfObjectState::SharedPtr selfObject) {
    this->selfObject = selfObject;
}

std::shared_ptr<std::unordered_map<uint32_t, Object>> WorldModel::get_history(bool friendly) {
    if(friendly){
        return this->friendObjects;
    }
    return this->enemyObjects;
}
