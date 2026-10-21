#include "t800_waypoint_rviz_plugins/waypoint_panel.hpp"

#include <functional>

#include <QGridLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace t800_waypoint_rviz_plugins
{

WaypointPanel::WaypointPanel(QWidget * parent)
: rviz_common::Panel(parent), status_label_(new QLabel("Waiting for waypoint editor", this))
{
  auto * root = new QVBoxLayout(this);
  auto * edit_group = new QGroupBox("Waypoint Editing", this);
  auto * edit_layout = new QGridLayout(edit_group);
  auto * save = new QPushButton("Save Route", edit_group);
  auto * reload = new QPushButton("Reload", edit_group);
  auto * delete_last = new QPushButton("Delete Last", edit_group);
  auto * clear = new QPushButton("Clear Route", edit_group);
  edit_layout->addWidget(save, 0, 0);
  edit_layout->addWidget(reload, 0, 1);
  edit_layout->addWidget(delete_last, 1, 0);
  edit_layout->addWidget(clear, 1, 1);

  auto * mission_group = new QGroupBox("Inspection Mission", this);
  auto * mission_layout = new QGridLayout(mission_group);
  auto * start = new QPushButton("Start Mission", mission_group);
  auto * cancel = new QPushButton("Stop Mission", mission_group);
  mission_layout->addWidget(start, 0, 0);
  mission_layout->addWidget(cancel, 0, 1);

  auto * control_group = new QGroupBox("Control Mode", this);
  auto * control_layout = new QGridLayout(control_group);
  auto * manual = new QPushButton("Manual Control", control_group);
  auto * automatic = new QPushButton("Auto Navigation", control_group);
  control_layout->addWidget(manual, 0, 0);
  control_layout->addWidget(automatic, 0, 1);

  status_label_->setWordWrap(true);
  status_label_->setMinimumHeight(45);
  root->addWidget(edit_group);
  root->addWidget(mission_group);
  root->addWidget(control_group);
  root->addWidget(status_label_);
  root->addStretch();

  connect(save, &QPushButton::clicked, this, &WaypointPanel::saveRoute);
  connect(reload, &QPushButton::clicked, this, &WaypointPanel::reloadRoute);
  connect(delete_last, &QPushButton::clicked, this, &WaypointPanel::deleteLast);
  connect(clear, &QPushButton::clicked, this, &WaypointPanel::clearRoute);
  connect(start, &QPushButton::clicked, this, &WaypointPanel::startMission);
  connect(cancel, &QPushButton::clicked, this, &WaypointPanel::cancelMission);
  connect(manual, &QPushButton::clicked, this, &WaypointPanel::setManualControl);
  connect(automatic, &QPushButton::clicked, this, &WaypointPanel::setAutoControl);
}

void WaypointPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("Unable to access the RViz ROS node", false);
    return;
  }
  node_ = abstraction->get_raw_node();
  save_client_ = node_->create_client<Trigger>("/t800/waypoints/save");
  reload_client_ = node_->create_client<Trigger>("/t800/waypoints/reload");
  delete_client_ = node_->create_client<Trigger>("/t800/waypoints/delete_last");
  clear_client_ = node_->create_client<Trigger>("/t800/waypoints/clear");
  start_client_ = node_->create_client<Trigger>("/t800/mission/start");
  cancel_client_ = node_->create_client<Trigger>("/t800/mission/cancel");
  manual_client_ = node_->create_client<Trigger>("/t800/control/set_manual");
  auto_client_ = node_->create_client<Trigger>("/t800/control/set_auto");
  events_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/t800/mission/events", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      const QString status = "Mission event: " + QString::fromStdString(message->data);
      QMetaObject::invokeMethod(
        this, [this, status]() {setStatus(status, true);}, Qt::QueuedConnection);
    });
  setStatus("Panel ready; start the waypoint editor", true);
}

void WaypointPanel::callService(const Client::SharedPtr & client, const QString & operation)
{
  if (!client || !client->service_is_ready()) {
    setStatus(operation + " failed: node or service is unavailable", false);
    return;
  }
  setStatus(operation + " in progress...", true);
  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
    request,
    [this, operation](Client::SharedFuture future) {
      const auto response = future.get();
      const QString message = operation + (response->success ? " succeeded: " : " failed: ") +
        QString::fromStdString(response->message);
      QMetaObject::invokeMethod(
        this,
        [this, message, success = response->success]() {setStatus(message, success);},
        Qt::QueuedConnection);
    });
}

void WaypointPanel::setStatus(const QString & message, bool success)
{
  status_label_->setText(message);
  status_label_->setStyleSheet(success ? "QLabel { color: #62d26f; }" :
    "QLabel { color: #ef6b73; }");
}

void WaypointPanel::saveRoute() {callService(save_client_, "Save route");}
void WaypointPanel::reloadRoute() {callService(reload_client_, "Reload route");}
void WaypointPanel::deleteLast() {callService(delete_client_, "Delete waypoint");}
void WaypointPanel::startMission() {callService(start_client_, "Start mission");}
void WaypointPanel::cancelMission() {callService(cancel_client_, "Stop mission");}
void WaypointPanel::setManualControl() {callService(manual_client_, "Manual control");}
void WaypointPanel::setAutoControl() {callService(auto_client_, "Auto navigation");}

void WaypointPanel::clearRoute()
{
  if (QMessageBox::question(
      this, "Confirm Clear", "Delete all waypoints from the current route?",
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
  {
    callService(clear_client_, "Clear route");
  }
}

}  // namespace t800_waypoint_rviz_plugins

PLUGINLIB_EXPORT_CLASS(t800_waypoint_rviz_plugins::WaypointPanel, rviz_common::Panel)
