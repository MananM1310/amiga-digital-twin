
```markdown
# AMIGA Digital Twin – Autonomous Weed Monitoring

This repository contains my **digital twin setup for the AMIGA Smart Farming robot**, focused on:

- 🌱 **Row-following using vision**
- 🛑 **Weed detection and stopping logic**
- 🗺️ Simulation-based experimentation for future real-world deployment
- 🛰️ Foundations for mapping and coverage planning

All development is done on a **clean separate digital twin repo** to avoid breaking the original AMIGA repository.

---

## 📂 Repository Structure

```

amiga_digital_twin/
│
├── src/
│   ├── AMIGA_Smart_Farming/     # Submodule (my fork)
│   ├── amiga_autonomy/          # My autonomy stack
│   └── amiga_state_estimation/  # State estimation (EKF experiments)
│
├── build/       # ignored
├── devel/       # ignored
├── .gitignore
├── .gitmodules
└── README.md

```

---

## ✅ Key Features Implemented

### 1. Vision-Based Row Following
- Uses **downward-facing Oak-D camera** (`oak_d_camera_1`).
- Detects crop row using green segmentation.
- Fits a line using detected plant clusters.
- Tracks the row and corrects steering.

Features:
- Handles sensor noise
- Handles missing detections
- Smooth steering using proportional control

---

### 2. Weed Stopping Logic
Integrated into the same node as row following.

Logic:
- Subscribes to `/sim/nearest_weed`
- Uses `/gazebo/model_states` to compute robot → weed distance
- Stops when within threshold distance
- Remembers visited weeds to avoid stopping again

---

### 3. Lane Visualization
Publishes animated curved lane overlay showing:
- Extracted row line
- Lane width region
- Detected plant points
- Weed stopping states

Debug topic:
```

/row_follow/front_debug

````

View using:
```bash
rosrun rqt_image_view rqt_image_view
````

---

## 🚜 Main Node: `row_follow_camera.py`

Location:

```bash
src/amiga_autonomy/scripts/row_follow_camera.py
```

Run with:

```bash
rosrun amiga_autonomy row_follow_camera.py \
  _image_topic:=/oak_d_camera_1/image_raw \
  _forward_speed:=0.3 \
  _kp_row:=0.4 \
  _lane_width_px:=70 \
  _use_weed_stopping:=true \
  _stop_distance:=1.5 \
  _stop_duration:=3.0 \
  _debug_image_topic:=/row_follow/front_debug
```

---

## 🌱 Weed Generation System

Weeds and plants are generated using a random scaling technique:

* Small plants (`scale <= 0.3`) → considered weeds
* Larger ones → crop plants

Located in:

```
src/AMIGA_Smart_Farming/src/amiga_sim/scripts/gen_weeds.py
```

Weed positions saved into:

```
src/AMIGA_Smart_Farming/src/amiga_sim/models/farm_weeds/weed_positions.json
```

Weed oracle publishes nearest weed:

```
/sim/nearest_weed
```

---

## 🧠 Design Philosophy

This stack is built with real deployment in mind:

* Simulation → Real robot pipeline
* Same structure for ROS nodes
* Same perception logic
* Same topics

Only sensor sources change between sim and real robot.

---

## ⚙️ Setup Instructions

Clone your repo:

```bash
git clone https://github.com/MananM1310/amiga-digital-twin.git
cd amiga-digital-twin
```

Initialize submodules:

```bash
git submodule update --init --recursive
```

Source workspace:

```bash
source devel/setup.bash
```

Launch simulation:

```bash
roslaunch amiga_sim sim.launch
```

---

## 🔭 Current Roadmap

* [x] Simulation startup
* [x] Vision-based row following
* [x] Weed stopping logic
* [x] Lane visualization
* [ ] Coverage mapping using GPS
* [ ] Unvisited area planning
* [ ] YOLO integration (sim + real)
* [ ] Transfer to real AMIGA robot

---

## 👤 Author

**Manan Maheshwari**
MS Autonomy & Robotics – UIUC
GitHub: [https://github.com/MananM1310](https://github.com/MananM1310)
Digital Twin Repo: [https://github.com/MananM1310/amiga-digital-twin](https://github.com/MananM1310/amiga-digital-twin)

---

## 📜 Notes

* This repository does **not modify** the original AMIGA repo directly.
* All enhancements live inside my digital twin layer.
* Safe for experimentation and extension.

---

🛠️ If you're building something similar, feel free to fork or reach out!


