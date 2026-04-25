# ROS2 SLAM + AMCL Localization Project

## Current Progress: Step 7 Complete ✅
- ✅ Step 1: Robot URDF Model  
- ✅ Step 2: Gazebo Integration & Basic Physics (Robot visible, physics working)
- ✅ Step 3: Differential Drive Control Plugin (Robot can move!)
- ✅ Step 4: LiDAR Sensor Integration (Robot can sense environment!)
- ✅ Step 5: Test Environment Creation (Robot has obstacles to detect!)
- ✅ Step 6: SLAM Toolbox Integration (Robot can build maps!)
- ✅ Step 7: Map Building & Saving (Robot has a permanent map!)
- ⏳ Step 8: Map Loading (Next)

---

## Quick Start

### Build the Project
```bash
cd /home/mohamed/test
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Run the Simulation (3 Terminals)

**Terminal 1: Start Gazebo Simulation (Headless)**
```bash
cd /home/mohamed/test
source install/setup.bash
ros2 launch robot_simulation gazebo_headless.launch.py
```

**Terminal 2: Start RViz Visualization**
```bash
cd /home/mohamed/test
unset GTK_PATH
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d src/robot_simulation/config/sim.rviz
```

**Terminal 3: Control with Keyboard**
```bash
cd /home/mohamed/test
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
- `i` = Forward
- `,` = Backward
- `j` = Turn left (rotate in place)
- `l` = Turn right (rotate in place)
- `k` = Stop
- `q/z` = Increase/decrease max speeds
- `Space` = Emergency stop

*Note: This is a differential drive robot - it cannot strafe (move sideways). It must rotate to change direction.*

---

## Test Robot Movement

## Test Robot Movement

**Verify the robot is moving in RViz (Terminal 2) while controlling from Terminal 3.**

You can also test manually:
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Project Structure
```
/home/mohamed/test/
├── src/
│   ├── robot_description/          # Robot URDF model
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro
│   │   │   └── robot.gazebo.xacro
│   │   ├── launch/
│   │   └── config/
│   └── robot_simulation/           # Gazebo simulation
│       ├── launch/
│       │   ├── gazebo.launch.py
│       │   └── gazebo_headless.launch.py
│       └── worlds/
│           ├── empty.world
│           └── test_world.world
├── install/                        # Built packages
├── build/                          # Build files
└── README.md                       # This file
```

---

## Robot Specifications
- **Type**: Differential drive (2 wheels + caster)
- **Dimensions**: 60cm × 60cm × 40cm (L × W × H)
- **Wheels**: 10cm radius, 50cm separation
- **Mass**: 15kg total
- **Max Speed**: 1.39 m/s (5 km/h)
- **Sensors**: LDRobot LD06 TOF LiDAR (0.50-12m range, 360° FOV, 10 Hz, 450 samples)

## Test Environment
- **Type**: Enclosed room with obstacles
- **Size**: 10m × 10m (100 m²)
- **Walls**: 4 outer walls + 2 interior walls
- **Obstacles**: 2 boxes + 2 cylinders (varied sizes)
- **Purpose**: LiDAR testing and SLAM mapping

---

## Development Roadmap (12 Steps)
1. ✅ Robot URDF Model Creation
2. ✅ Gazebo Integration & Basic Physics  
3. ✅ **Differential Drive Control Plugin**
4. ✅ **LiDAR Sensor Integration**
5. ✅ **Test Environment Creation**
6. ✅ **SLAM Toolbox Integration**
7. ✅ **Map Building & Saving**
8. ⏳ **Map Loading** ← Next
9. ⏳ AMCL Integration
10. ⏳ AMCL Testing
11. ⏳ Autonomous Navigation
12. ⏳ Complex Environment

---

## Troubleshooting

### RViz doesn't launch
**Error:** `qt.qpa.plugin: Could not find the Qt platform plugin "wayland"`  
**Solution:** Use the `unset` commands before launching RViz (see Terminal 2 above)

### Robot doesn't move
1. Check simulation is running (Terminal 1 should show Gazebo logs)
2. Verify topics exist: `ros2 topic list` (should show `/cmd_vel` and `/odom`)
3. Test directly: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"`

### Kill stuck Gazebo
```bash
killall -9 gzserver gzclient
```

### Rebuild project
```bash
cd /home/mohamed/test
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

*For detailed troubleshooting history, see `TROUBLESHOOTING_LOG.md`*

---

## VirtualBox Note

This project uses **Gazebo headless + RViz** for visualization because:
- VirtualBox's OpenGL has limitations with Gazebo GUI
- This is the **standard professional workflow** for ROS2 robotics
- RViz provides better visualization for SLAM/AMCL than Gazebo anyway

---

## Notes
- Use **native terminal** (outside VS Code) for GUI applications
- All steps (SLAM, mapping, navigation) work perfectly with this setup

---

**Project Start**: October 15, 2025  
**Status**: On Track ✅
