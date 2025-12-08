# dragoman_fsw_sim

Flight software simulators for dragoman telemetry and command handling.

This package contains simulation scripts, launch files, and utilities that were moved from `dragoman_sandbox` to better organize the codebase.

## Python Modules

This package includes the following Python modules in `dragoman_fsw_sim/`:

- **clr_trajectory_router.py** - Trajectory routing utilities for CLR robot
- **curiosity_xtce_construct_generator.py** - XTCE construct generator for Curiosity rover telemetry
- **imetro_xtce_construct_generator.py** - XTCE construct generator for iMetro demo telemetry
- **srdf_parser.py** - Parser for SRDF (Semantic Robot Description Format) files

## Simulators

This package includes the following simulator scripts:

- **curiosity_simple_simulator.py** - Simulator for Curiosity rover telemetry and commands
- **imetro_demo_dummy_simulator.py** - Dummy simulator with hardcoded telemetry for iMetro demo
- **imetro_demo_robot_simple_simulator.py** - Robot simulator for iMetro demo with real robot integration
- **all_types_simulator.py** - Simulator demonstrating all XTCE parameter types
- **multipacket_simulator.py** - Multi-packet telemetry simulator with different APIDs
- **move_robot.py** - Script for moving robot joints and testing robot control

## Behavior Trees

This package includes behavior tree definitions for CLR robot operations:

- **behaviors/imetro_clr_pnp_cargo_bag.xml** - Pick and place behavior for cargo bag handling
- **behaviors/imetro_clr_take_pictures.xml** - Behavior for taking pictures with CLR robot

## Launch Files

### Simulation Launch Files
- **curiosity_simulation.launch.py** - Launch Curiosity rover simulation with Gazebo
- **imetro_dummy_demo.launch.py** - Launch iMetro dummy demo with hardcoded telemetry
- **imetro_robot_simple_demo.launch.py** - Launch iMetro robot demo with real robot

### Test and Visualization Launch Files
- **test_clr.launch.py** - Test launch file for CLR (Chonkur L Raile) robot with MuJoCo simulation
- **test_robot.launch.py** - Test launch file for generic robot with MuJoCo simulation
- **view_clr.launch.py** - Launch CLR robot visualization with RViz
- **view_clr_trainer_multi_hatch.launch.py** - Launch CLR trainer with multi-hatch configuration and RViz

## Configuration Files

- **config/curiosity.srdf** - Semantic Robot Description Format file for Curiosity rover (defines robot groups and states)

## RViz Configuration Files

- **rviz/clr_sim.rviz** - RViz configuration for CLR robot simulation visualization
- **rviz/mujoco_test_clr.rviz** - RViz configuration for CLR robot MuJoCo testing
- **rviz/mujoco_test_robot.rviz** - RViz configuration for generic robot MuJoCo testing

## URDF Files

- **urdf/test_clr_xacro.urdf** - URDF file for CLR robot testing with xacro

## Usage Examples

### Run Dummy Demo (Hardcoded Telemetry)

1. Start the YAMCS server with iMetro configuration:
   ```bash
   ros2 run dragoman_yamcs_project dragoman_yamcs
   ```

2. Open YAMCS web interface in a browser:
   ```
   http://localhost:8090/
   ```
   You should see the YAMCS Mission Control with the iMetro interface loaded.

3. Launch the demo script that publishes telemetry data and reads commands (spacecraft side):
   ```bash
   ros2 launch dragoman_fsw_sim imetro_dummy_demo.launch.py
   ```
   *This runs in ROS_DOMAIN_ID=100 to isolate ROS2 communication from the ground side.*

4. (Optional) Launch YAMCS ↔ ROS2 bridge and RViz visualization (ground side):
   ```bash
   ros2 launch dragoman_ground_example imetro_ground.launch.py
   ```
   *This runs in default ROS_DOMAIN_ID=0 to isolate ROS2 communication from the spacecraft side.*

5. Use YAMCS to view telemetry being sent from the script and send commands.

### Run Simple Robot Demo (iMetro Setup)

1. Start the YAMCS server with iMetro configuration:
   ```bash
   ros2 run dragoman_yamcs_project dragoman_yamcs
   ```

2. Open YAMCS web interface in a browser:
   ```
   http://localhost:8090/
   ```
   You should see the YAMCS Mission Control with the iMetro interface loaded.

3. Launch the robot demo script (spacecraft side):
   ```bash
   ros2 launch dragoman_fsw_sim imetro_robot_simple_demo.launch.py
   ```
   *This runs in ROS_DOMAIN_ID=100 to isolate ROS2 communication from the ground side.*

4. (Optional) Launch YAMCS ↔ ROS2 bridge and RViz visualization (ground side):
   ```bash
   ros2 launch dragoman_ground_example imetro_ground.launch.py
   ```
   *This runs in default ROS_DOMAIN_ID=0 to isolate ROS2 communication from the spacecraft side.*

5. Use YAMCS to view telemetry from the robot and control the arm and lift/rail joints.

### Run Simple Robot Demo (Curiosity Setup)

1. Start the YAMCS server with Curiosity configuration:
   ```bash
   ros2 run dragoman_yamcs_project dragoman_yamcs
   ```

2. Open YAMCS web interface in a browser:
   ```
   http://localhost:8090/
   ```
   You should see the YAMCS Mission Control with the Curiosity interface loaded.

3. Launch the Gazebo simulation (spacecraft side):
   ```bash
   ros2 launch dragoman_fsw_sim curiosity_simulation.launch.py
   ```
   *This runs in ROS_DOMAIN_ID=100 to isolate ROS2 communication from the ground side.*

4. (Optional) Launch YAMCS ↔ ROS2 bridge and RViz visualization (ground side):
   ```bash
   ros2 launch dragoman_ground_example curiosity_ground.launch.py
   ```
   *This runs in default ROS_DOMAIN_ID=0 to isolate ROS2 communication from the spacecraft side.*

## Dependencies

This package depends on:
- `dragoman_sandbox` - For shared utilities and XTCE construct generators
- `dragoman_yamcs_project` - For YAMCS server configurations
- `dragoman_generated_msgs` - For custom telemetry message definitions
- ROS2 packages: `rclpy`, `sensor_msgs`, `trajectory_msgs`, `control_msgs`, etc.

## Related Packages

- [`dragoman_sample_xtce`](../dragoman_sample_xtce/README.md) - Generate XTCE files for telemetry definitions
- [`dragoman_yamcs_project`](../dragoman_yamcs_project/dragoman_yamcs_project/README.md) - YAMCS server configuration and setup
- [`dragoman_yamcs_ros_bridge`](../dragoman_yamcs_ros_bridge/README.md) - Bridge between YAMCS and ROS2
