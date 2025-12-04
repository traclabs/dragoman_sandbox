# README #


### Generate an XTCE file ####

* You can generate the XTCE we are using for the iMetro demo running the following:

  ```
  ros2 run dragoman_sandbox pymdb_generate_imetro_demo_xtce.py
  ```

  This will generate the XTCE file in the install/share folder of dragoman_sandbox/xtce.
  For it to be used by YAMCS, you'll have to copy and paste it to dragoman_sandbox/imetro_yamcs_project
  (in src/main/yamcs/mdb).


### Run dummy demo (hardcoded telemetry) ###

* Run our imetro_yamcs project that has an XTCE defined for some ROS2 tools (first time may take a while to download dependencies):

   ```
   ros2 run dragoman_sandbox imetro_yamcs
   ```

* In a web browser, open the terminal at: http://localhost:8090/ . You sould see the YAMCS Mission Control with the imetro interface loaded up.

* Run our demo script that publishes telemetry data and reads commands:

  ```
  ros2 launch dragoman_sandbox imetro_dummy_demo.launch.py
  ```

* Use YAMCS to see the telemetry being sent from our script, and send commands.

### Run simple robot demo (iMetro setup) ###

* Run our imetro_yamcs project that has an XTCE defined for some ROS2 tools:

   ```
   ros2 run dragoman_sandbox imetro_yamcs
   ```

* In a web browser, open the terminal at: http://localhost:8090/ . You sould see the YAMCS Mission Control with the imetro interface loaded up.

* Run our demo script that publishes telemetry data and reads commands:

  ```
  ros2 launch dragoman_sandbox imetro_robot_simple_demo.launch.py
  ```

* Use YAMCS to see the telemetry being sent from our script (from the robot) and move the arm and the lift/rail joints

### Run simple robot demo (Curiosity setup) ###

* Run our curiosity_yamcs project that has an XTCE defined for some ROS2 tools:

   ```
   ros2 run dragoman_sandbox curiosity_yamcs
   ```

* In a web browser, open the terminal at: http://localhost:8090/ . You sould see the YAMCS Mission Control with the imetro interface loaded up.

* Run our demo script that publishes telemetry data and reads commands:

  ```
  ros2 launch dragoman_sandbox curiosity_simple_demo.launch.py
  ```

* Use YAMCS to see the telemetry being sent from our script (from the robot) and move the arm and the mast joints
