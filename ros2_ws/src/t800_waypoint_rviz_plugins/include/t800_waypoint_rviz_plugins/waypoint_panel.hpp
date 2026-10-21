#ifndef T800_WAYPOINT_RVIZ_PLUGINS__WAYPOINT_PANEL_HPP_
#define T800_WAYPOINT_RVIZ_PLUGINS__WAYPOINT_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace t800_waypoint_rviz_plugins
{

class WaypointPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void saveRoute();
  void reloadRoute();
  void deleteLast();
  void clearRoute();
  void startMission();
  void cancelMission();
  void setManualControl();
  void setAutoControl();

private:
  using Trigger = std_srvs::srv::Trigger;
  using Client = rclcpp::Client<Trigger>;

  void callService(const Client::SharedPtr & client, const QString & operation);
  void setStatus(const QString & message, bool success);

  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr save_client_;
  Client::SharedPtr reload_client_;
  Client::SharedPtr delete_client_;
  Client::SharedPtr clear_client_;
  Client::SharedPtr start_client_;
  Client::SharedPtr cancel_client_;
  Client::SharedPtr manual_client_;
  Client::SharedPtr auto_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_subscription_;
  QLabel * status_label_;
};

}  // namespace t800_waypoint_rviz_plugins

#endif  // T800_WAYPOINT_RVIZ_PLUGINS__WAYPOINT_PANEL_HPP_
