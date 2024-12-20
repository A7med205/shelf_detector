#### Dependencies
```
Nav2
pip install boto3 #for voice control
pip install assemblyai #for voice control
```

#### Launch files
```
# SIM
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml

# SIM control
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml sim_time:=true amcl_yaml:=amcl_config_sim.yaml
ros2 launch path_planner_server pathplanner.launch.py sim_time:=true teleop_topic:=diffbot_base_controller/cmd_vel_unstamped

# Webapp
ros2 launch shelf_detector shelf_detector_launch.py mode:=0
ros2 launch shelf_detector web_services_launch.py web_dir:=/home/user/webpage_ws/detector_app
---------------------------------------------------------------
# REAL robot control

ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml sim_time:=false amcl_yaml:=amcl_config_real.yaml
ros2 launch path_planner_server pathplanner_real.launch.py sim_time:=false teleop_topic:=/cmd_vel

# Webapp
ros2 launch shelf_detector shelf_detector_launch.py mode:=1
ros2 launch shelf_detector web_services_launch_8.py web_dir:=/home/user/webpage_ws/detector_app_1
---------------------------------------------------------------
```
* Webpage_ws files can be found at https://github.com/A7med205/webpage_final
* Real robot rosbridge starts using port 8080 instead of 9090
* copy the new rosbridge address to the webpage 

#### Webapp sections:
- Connection area
- The control panel
- Manual control joystick
- Map with localized robot element
- Status area
- Log area

#### Get addresses
- Address for the webpage should be http://localaddress:7000
- Rosbridge address should be ws://localhost:8080 
