# Waypoint Navigation - Usage Guide

## Overview

The waypoint navigator allows you to command the AMIGA robot to drive to specific (x, y) coordinates in the Gazebo world frame. The robot will autonomously navigate using a two-stage control strategy:
1. **Rotate** to face the goal
2. **Drive forward** while maintaining heading

## Quick Start

### 1. Launch the Simulation with Waypoint Navigator

```bash
# In your ROS Docker container
cd /host/home/manan/amiga_digital_twin
source devel/setup.bash
roslaunch amiga_autonomy waypoint_nav.launch
```

This will start:
- Gazebo simulation with the AMIGA robot and farm environment
- RViz visualization
- State estimation (odometry and IMU)
- Waypoint navigator node

### 2. Send a Goal

In a new terminal, send a goal coordinate:

```bash
# Navigate to coordinates (x=5.0, y=10.0)
rostopic pub /waypoint_nav/goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 10.0, z: 0.0}}
}'
```

The robot will:
1. Rotate to face the goal
2. Drive forward to the goal
3. Stop when within tolerance (default: 0.3m)

### 3. Monitor Status

```bash
# Watch navigation status
rostopic echo /waypoint_nav/status

# Watch robot position
rostopic echo /sim/wheel_odom
```

Status values:
- `IDLE`: No goal, waiting
- `ROTATING`: Turning to face goal
- `DRIVING`: Moving toward goal
- `REACHED`: Goal achieved

## Example Goals

```bash
# Goal 1: Drive forward 5 meters
rostopic pub /waypoint_nav/goal geometry_msgs/PoseStamped '{header: {frame_id: "odom"}, pose: {position: {x: 3.0, y: -20.0, z: 0.0}}}'

# Goal 2: Move to different row (1.18m row spacing)
rostopic pub /waypoint_nav/goal geometry_msgs/PoseStamped '{header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: -15.0, z: 0.0}}}'

# Goal 3: Return to start
rostopic pub /waypoint_nav/goal geometry_msgs/PoseStamped '{header: {frame_id: "odom"}, pose: {position: {x: -1.87, y: -20.0, z: 0.0}}}'
```

## Configuration Parameters

Edit the launch file to adjust navigation behavior:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | 0.5 m/s | Forward driving speed |
| `angular_speed` | 0.3 rad/s | Rotation speed |
| `goal_tolerance` | 0.3 m | Distance to consider goal reached |
| `heading_tolerance` | 0.1 rad (~6°) | Heading error before rotating in place |
| `kp_heading` | 1.0 | Proportional gain for heading correction |

Example with custom parameters:

```xml
<node pkg="amiga_autonomy" type="waypoint_navigator.py" name="waypoint_navigator" output="screen">
  <param name="linear_speed" value="0.8" />  <!-- Faster -->
  <param name="goal_tolerance" value="0.2" />  <!-- More precise -->
</node>
```

## Troubleshooting

### Robot doesn't move
- Check that odometry is being published: `rostopic hz /sim/wheel_odom`
- Verify goal was received: `rostopic echo /waypoint_nav/goal`
- Check navigator logs: Look for "New goal received" message

### Robot spins in place
- The heading tolerance may be too tight
- Try increasing `heading_tolerance` to 0.2 or higher

### Robot drives straight instead of turning
- **Kinematic Mismatch**: This usually means the wheel commands are inverted relative to the physical simulation.
- **Fix**: Check `sim_control.py` and ensure right wheel commands are inverted (`-v_right`) if the URDF axis orientation requires it.
- **Friction**: If wheels can't slip, skid steering fails. Ensure URDF wheel friction (`mu`, `mu2`) is low (e.g., 1.0/0.5).

### Robot overshoots goal
- Reduce `linear_speed` for better stopping accuracy
- Decrease `goal_tolerance` for tighter final positioning

### Robot drives in circles
- Check `kp_heading` parameter - if too high, oscillations occur
- Reduce to 0.5 or lower for smoother motion

## Advanced: Sequential Waypoints

To navigate multiple waypoints in sequence, you can create a simple script:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

goals = [(5.0, 10.0), (10.0, 5.0), (-1.87, -20.0)]
current_goal = 0
status = "IDLE"

def status_callback(msg):
    global status
    status = msg.data

rospy.init_node("waypoint_sequence")
pub = rospy.Publisher("/waypoint_nav/goal", PoseStamped, queue_size=10)
rospy.Subscriber("/waypoint_nav/status", String, status_callback)

rate = rospy.Rate(1)
while not rospy.is_shutdown() and current_goal < len(goals):
    if status == "REACHED" or status == "IDLE":
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = goals[current_goal][0]
        goal.pose.position.y = goals[current_goal][1]
        pub.publish(goal)
        rospy.loginfo(f"Sent goal {current_goal+1}: {goals[current_goal]}")
        current_goal += 1
        rospy.sleep(2)
    rate.sleep()
```

## Next Steps

- **Path Planning**: Integrate with ROS navigation stack (move_base)
- **Obstacle Avoidance**: Add costmaps and local planners
- **Interactive Goals**: Use RViz to click waypoints on the map
