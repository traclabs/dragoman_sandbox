Generate an XTCE file
======================

* You can generate the XTCE we are using for the iMetro demo running the following:

  ```
  ros2 run dragoman_sandbox pymdb_generate_imetro_demo_xtce.py
  ```

  This will generate the XTCE file in the install/share folder of dragoman_sandbox/xtce.
  For it to be used by YAMCS, you'll have to copy and paste it to dragoman_sandbox/imetro_yamcs_project
  (in src/main/yamcs/mdb).
