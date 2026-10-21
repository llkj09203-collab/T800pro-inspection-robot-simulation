#include "t800_waypoint_rviz_plugins/add_waypoint_tool.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace t800_waypoint_rviz_plugins
{

AddWaypointTool::AddWaypointTool()
: topic_property_(new rviz_common::properties::StringProperty(
      "Topic", "/t800/waypoints/add_pose",
      "Pose topic used to add T800 waypoints.",
      getPropertyContainer(), SLOT(updateTopic()), this))
{
  shortcut_key_ = 'a';
  setName("Add Waypoint");
}

void AddWaypointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  setName("Add Waypoint");
  updateTopic();
}

void AddWaypointTool::updateTopic()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 10);
}

void AddWaypointTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(theta / 2.0);
  pose.pose.orientation.w = std::cos(theta / 2.0);
  publisher_->publish(pose);
}

}  // namespace t800_waypoint_rviz_plugins

PLUGINLIB_EXPORT_CLASS(t800_waypoint_rviz_plugins::AddWaypointTool, rviz_common::Tool)
