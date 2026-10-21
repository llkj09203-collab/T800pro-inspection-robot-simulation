#ifndef T800_WAYPOINT_RVIZ_PLUGINS__ADD_WAYPOINT_TOOL_HPP_
#define T800_WAYPOINT_RVIZ_PLUGINS__ADD_WAYPOINT_TOOL_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

namespace t800_waypoint_rviz_plugins
{

class AddWaypointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  AddWaypointTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * topic_property_;
};

}  // namespace t800_waypoint_rviz_plugins

#endif  // T800_WAYPOINT_RVIZ_PLUGINS__ADD_WAYPOINT_TOOL_HPP_
