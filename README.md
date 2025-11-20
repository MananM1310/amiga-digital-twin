This repository is a ROS Noetic catkin workspace that wraps:

- **AMIGA_Smart_Farming** (simulation): farm-ng AMIGA robot + horseradish field + weed generator
- **amiga-ros-bridge**: official farm-ng ROS bridge
- **amiga_autonomy**: my own autonomy package for simulation and, later, real robot deployment

## Structure

```text
amiga_digital_twin/
├── src/
│   ├── AMIGA_Smart_Farming/      # fork (branch: simulation)
│   │   └── src/amiga_sim/        # Gazebo sim, weeds, RViz config, etc.
│   ├── amiga-ros-bridge/         # ROS bridge from farm-ng
│   └── amiga_autonomy/           # custom autonomy nodes
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── scripts/
│           ├── row_go_straight.py
│           └── row_follow_camera.py
└── README.md
Features
1. Row-following from downward camera
Node: amiga_autonomy/scripts/row_follow_camera.py

Subscribes to: /oak_d_camera_1/image_raw

Rotates the image so ground is at the bottom.

Segments green plants in HSV, samples green points across the whole image.

Fits a line to plant centers and builds a lane-shaped band (not just a stiff center line).

Computes steering command to keep the AMIGA centered in the row.

Publishes velocity commands on: /amiga/cmd_vel

Publishes a debug image with the lane overlay on: /row_follow/front_debug

2. Simulation weeds with consistent positions
Weeds are generated in AMIGA_Smart_Farming:

Script: src/AMIGA_Smart_Farming/src/amiga_sim/scripts/gen_weeds.py

SDF output: src/AMIGA_Smart_Farming/src/amiga_sim/models/farm_weeds/model.sdf

Also writes a JSON file of weed positions:
src/AMIGA_Smart_Farming/src/amiga_sim/models/farm_weeds/weed_positions.json

We use the scale of each plant:

Small plants (scale ≤ 0.3) are treated as weeds.

Larger plants are treated as regular crop plants.

Only weeds on the robot’s row are considered for stopping.

3. Weed “oracle” and stopping logic
Simulation-only “oracle” node:

Script: src/AMIGA_Smart_Farming/src/amiga_sim/scripts/sim_weed_oracle.py

Subscribes to /gazebo/model_states and weed_positions.json

Publishes the closest weed in the world frame as a PoseStamped on:
/sim/nearest_weed

The row-follow controller:

Reads /sim/nearest_weed and the robot pose from /gazebo/model_states.

Maintains a list of visited weeds, so it doesn’t stop at the same weed twice.

Behaviour:

Drive forward while following the row.

If a new weed ahead is within _stop_distance, stop for _stop_duration seconds.

Then continue driving and move on to the next weed.

4. Fake GPS for later mapping / coverage
A simple fake GPS node (simulation only):

Script: src/AMIGA_Smart_Farming/src/amiga_sim/scripts/sim_fake_gps.py

Converts the robot’s world pose into a fake GPS-like message on /sim/gps/fix.

Intended to be replaced with real GPS on the physical AMIGA.

How to run the simulation
From your ROS Noetic Docker container:

Source ROS and the workspace:

bash
Copy code
cd /host/home/manan/amiga_digital_twin
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
Launch the AMIGA simulation (Gazebo + weeds + RViz):

bash
Copy code
roslaunch amiga_sim sim.launch
In a new terminal, start the row-follow controller with weed stopping:

bash
Copy code
rosrun amiga_autonomy row_follow_camera.py \
  _image_topic:=/oak_d_camera_1/image_raw \
  _forward_speed:=0.3 \
  _kp_row:=0.4 \
  _lane_width_px:=90 \
  _use_weed_stopping:=true \
  _stop_distance:=1.5 \
  _stop_duration:=3.0 \
  _debug_image_topic:=/row_follow/front_debug
You should see:

The AMIGA driving along a plant row in Gazebo.

RViz showing the robot, weeds, and camera view.

Debug overlay topic /row_follow/front_debug with the lane band drawn over the plants.

Log messages printing the distance to the next weed and when the robot stops/starts.

Upstream vs fork
This workspace uses my fork of AMIGA_Smart_Farming:

https://github.com/MananM1310/AMIGA_Smart_Farming (branch: simulation)

The original author’s repo is kept as the upstream remote so I can pull updates:

https://github.com/Janmejay-Rathi/AMIGA_Smart_Farming

The idea is to keep:

upstream: clean, original version

origin (my fork): simulation + autonomy additions used for the digital twin