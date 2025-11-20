# Campus Navigation & Stairs Avoidance Architecture

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

---

## 1. Operating Environment Challenges (IIT Mandi Campus)

The IIT Mandi campus is built across mountainous terrain in the Kamand Valley. Key navigation considerations:
1. **Mountain Incline**: Walkways between hostel clusters, academic complexes, and dining halls have grades up to 10%–15% ($6^\circ$–$9^\circ$).
2. **Pedestrian Density**: Crowded paths during class changeovers require conservative velocity limits and dynamic obstacle avoidance.
3. **Negative Obstacles (Staircases & Cliffs)**: Multiple buildings feature pedestrian staircases directly adjacent to paved ramps.
4. **GPS Shadows**: Multi-story structures and canyon walls cause GNSS multipath and intermittent fix dropouts.

---

## 2. The Stairs Avoidance Solution

### Why LiDAR Alone Is Insufficient
A planar 2D LiDAR emits laser pulses in a single horizontal plane parallel to the chassis. As the robot approaches a downward staircase:
- The laser beams pass horizontally over the descending stairs into empty air.
- The costmap sees "free space" ahead because no return echo is reflected back.
- If unconstrained, the robot would drive straight off the cliff or down the stairs!

### The Multi-Tiered Engineering Strategy

```
               [Stairs Avoidance Multi-Tiered Architecture]
                                   |
    +------------------------------+------------------------------+
    |                                                             |
    v                                                             v
[Tier 1: Semantic Costmap Keepout Filter]          [Tier 2: Real-time Ground Sensors]
- Pre-mapped geometric keepout masks                - Downward-angled ultrasonic sensors
- Nav2 Costmap Filter plugin marks stairs             sense ground drops > 100mm
  with LETHAL_OBSTACLE (254) cost                   - Immediate hardware E-Stop trigger
- Planner will never route path across stairs       - Fallback against map misalignment
```

#### Tier 1: Semantic Costmap Keepout Filters
Using the ROS 2 Nav2 Costmap Filter pipeline:
1. When generating or editing the campus map, all known staircases, terrace edges, and ravines are shaded black (`0` or `100` cost) on a separate companion mask: `maps/stairs_keepout_mask.pgm`.
2. The `costmap_filter_info_server` and `filter_mask_server` publish this mask on `/stairs_keepout_mask`.
3. The `nav2_costmap_2d` global costmap imports this layer. Even if LiDAR reports free space, the costmap marks the region as an impassable physical wall, forcing the global path planner to route exclusively along approved ramps.

#### Tier 2: Real-time Downward Range Sensors
- Downward-angled sensors mounted on the front bumper detect distance to the pavement ($d_{nominal} \approx 120\text{ mm}$).
- If $d > 220\text{ mm}$ (indicating a step down, open drain, or pavement collapse), the sensor immediately asserts the hardware emergency stop relay.

#### Tier 3: IMU Pitch/Roll Hazard Stop
- If the robot ever encounters an unexpected step or steep incline exceeding $16^\circ$ pitch, the `mpu6050_node` issues a critical stop on `/emergency_stop`.

---

## 3. Nav2 Controller Configuration

The robot uses the **Regulated Pure Pursuit Controller** (`nav2_regulated_pure_pursuit_controller`):

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.50             # 0.50 m/s maximum campus cruise speed
  lookahead_dist: 0.8                  # meters
  min_lookahead_dist: 0.4
  max_lookahead_dist: 1.5
  use_velocity_scaled_lookahead_dist: true
  use_cost_regulated_linear_velocity_scaling: true
  regulated_linear_scaling_min_speed: 0.15
  use_rotate_to_heading: true          # Align with goal heading before translating
  max_angular_accel: 1.0               # rad/s^2 smooth turning on slopes
```

### Key Advantages:
- **Cost-regulated speed scaling**: Automatically slows down from 0.50 m/s to 0.15 m/s when passing near pedestrians or narrow corridor walls.
- **In-place rotation before translation**: Avoids wide sweeping turns that could drive outer wheels off narrow walkway edges.
- **Velocity ramping**: Prevents hot food containers and grocery cargo from sliding off the robot payload tray.

---

## 4. Mapping & SLAM Workflow

To generate a new campus map:

1. Launch base drivers and RPLIDAR:
   ```bash
   ros2 launch robot_bringup sensors.launch.py use_mock_hardware:=false
   ros2 launch robot_bringup base.launch.py use_mock_hardware:=false
   ```
2. Launch SLAM Toolbox:
   ```bash
   ros2 launch robot_navigation slam.launch.py
   ```
3. Drive the robot slowly along the target delivery route via teleoperation:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. Save the completed map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/autonomous_delivery_robot/maps/iit_mandi_campus
   ```
5. Open the saved `.pgm` file in an image editor (e.g. GIMP) to generate the corresponding `stairs_keepout_mask.pgm` by marking staircases and restricted terrain in solid black.
