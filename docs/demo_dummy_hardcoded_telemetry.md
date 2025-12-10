Run dummy demo (hardcoded telemetry)
====================================

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

