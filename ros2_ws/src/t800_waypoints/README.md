# T800 waypoint editing and missions

This package provides a T800-native replacement for the WPR `wp_map_tools`
workflow. It uses standard Nav2 actions and does not depend on `/waterplus/*`
topics.

## Edit and save a route

Start navigation, the editor and RViz:

```bash
ros2 launch t800_navigation navigation.launch.py
ros2 launch t800_waypoints waypoints.launch.py
ros2 launch t800_visualization rviz.launch.py
```

In RViz, select **Add Waypoint**, press on the map and drag to set the arrival
heading. Select **Interact** to move the marker with its two translation axes
or rotate its heading ring. New points are named `P01`, `P02`, and so on. The
keyboard shortcut for **Add Waypoint** is `A`.

Save the route explicitly:

```bash
ros2 service call /t800/waypoints/save std_srvs/srv/Trigger '{}'
```

The RViz **T800 Waypoints** panel provides the same operation through its
**保存路线** button, along with reload, delete, clear, start mission and stop
mission controls. The service commands remain available for automation.

The default writable route file is:

```text
~/.ros/t800_routes/mixed_inspection.yaml
```

Other editing services are:

```text
/t800/waypoints/reload
/t800/waypoints/delete_last
/t800/waypoints/clear
```

All use `std_srvs/srv/Trigger`. Pass `autosave:=true` to save every edit.

## Run the route

When using `waypoints.launch.py`, the mission manager is already waiting for
the RViz **Start Mission** button. The standalone automatic form is:

```bash
ros2 launch t800_waypoints mission.launch.py
```

The manager calls `NavigateToPose` for each waypoint and publishes JSON events
on `/t800/mission/events`. `none` and `wait` are built-in actions. Other action
names publish an `action_requested` event so camera or inspection nodes can be
connected without coupling them to the navigation package. `stay_seconds`
defines the dwell time before the next waypoint.

Cancel a mission with:

```bash
ros2 service call /t800/mission/cancel std_srvs/srv/Trigger '{}'
```

For manual start, launch with `autostart:=false`, then call
`/t800/mission/start`.

## Route format

```yaml
version: 1
map: mixed_map
frame_id: map
waypoints:
  - name: P01
    x: 2.5
    y: -1.2
    yaw: 1.57
    action: take_photo
    stay_seconds: 3.0
```
