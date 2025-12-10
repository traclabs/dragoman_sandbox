### Run simple robot demo (Curiosity setup) ###

* Run our curiosity_yamcs project that has an XTCE defined for some ROS2 tools:

      ```
      ros2 run dragoman_sandbox curiosity_yamcs
      ```

* In a web browser, open the terminal at: [http://localhost:8090/](http://localhost:8090/) . You sould see the YAMCS Mission Control with the **Curiosity** interface loaded up.

* Run the simulation with Gazebo (spacecraft side):

      ```
      ros2 launch dragoman_sandbox curiosity_simulation.launch.py
      ```
      
      *This runs in ROS_DOMAIN_ID=100 to isolate ROS2 communication with the ground side.*

* Run YAMCS <-> ROS2 bridge and RViz visualization (ground side, optional):

      ```
      ros2 launch dragoman_sandbox curiosity_ground_vis.launch.py
      ```

      *This runs in default ROS_DOMAIN_ID=0 to isolate ROS2 communication with the spacecraft side.*
  
 
![Curiosity sim](docs/images/curiosity_gz_demo.png){width=640 height=480}
 

