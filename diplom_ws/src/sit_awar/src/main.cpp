#include "SituationAwareness.h"
#include "WorldModel.h"
#include "DataHungarianAggregation.h"
#include "DataKalmanFilter.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("sit_awar", "agent0");

  std::shared_ptr<WorldModel> worldModel = std::make_shared<WorldModel>();
  std::shared_ptr<DataAggregation> dataAggregation = std::make_shared<DataHungarianAggregation>();
  std::shared_ptr<DataFilter> dataFilter = std::make_shared<DataKalmanFilter>();
  SituationAwareness sitAwar = SituationAwareness(worldModel, dataAggregation, dataFilter, node);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SituationAwareness::timer_callback, sitAwar));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
