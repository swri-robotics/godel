## Godel

Application for demonstrating surface blending with ROS.

Godel: Austrian logician and mathematician http://en.wikipedia.org/wiki/Kurt_G%C3%B6del

### Installation

- Install [wstool](http://wiki.ros.org/wstool) in order manage the repos inside the workspace
  ```
  sudo apt-get install python-wstool
  ```

- Cd into the 'src' directory of your catkin workspace and run the following:
  ```
  wstool init . 
  wstool merge https://github.com/ros-industrial-consortium/godel/raw/indigo-devel/godel.rosinstall
  wstool update
  rosdep install --from-paths . --ignore-src
  cd ..
  catkin_make
  ```
  
### Application can work at two modes:
-  Local mode
  -  All the functions run at the same machine.
-  Network mode
  -  Functions are distributed to two different machine: Server and Client.
  -  Server Machine:
    -  Runs perceptions nodes such as surface detection
    -  Runs motion planning nodes such as the process path planner and free motion planner (moveit).
  -  Client Machine:
    -  Runs Rviz for user front-end and visualization only.
    -  Runs either industrial_robot_simulator node or robot_driver node depending on whether it is on sim or real mode.
    -  Runs either generate_point_cloud node or Kinect sensor driver node also based on the mode being used.

  Note: One server can support multiple clients.
    
### Local mode

- Run blending demo in full simulation mode (simulated robot and sensor) 
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch
  ```

- Run blending demo in robot simulation mode (simulated robot and real sensor data)
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_sensor:=false
  ```

- Run blending demo in sensor simulation mode (real robot and simulated sensor)
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_robot:=false robot_ip:=[robot ip]
  ```

- Run blending demo in full real mode
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_sensor:=false 
  sim_robot:=false robot_ip:=[robot ip]
  ```

- Bring up Kinect2 sensor, if available:
  ```
  roslaunch godel_surface_detection kinect2.launch
  ```

### Network mode
- Run blending demo in full simulation mode (simulated robot and sensor)
  
  Server: 
  If it's the first RobotCell instance,run following program: 
  ```
  roslaunch godel_irb2400_support irb2400_blending_server.launch namespace:=[robot namespace] first_robot:="true"
  ```
  If it's not the first robotCell instance, run following program:
  ```
  roslaunch godel_irb2400_support irb2400_blending_server.launch namespace:=[robot namespace]
  ```
  Client:
  Run following program:
  ```
  rosrun godel_network irb2400_client.sh [server ip] [robot namespace] false
  ```
  To automatically run blending cycle (without maually interaction):
  ```
  rosrun godel_network irb2400_client.sh [server ip] [robot namespace] true
  ```
  Note:Robot namespace is the identity of individual robot, they can't be the same.
  
  
  
### Calibration
This section contains instructions for performing extrinsic calibration of a camera (on the robot end effector) to the robot arm. This is only necessary if you are running on real hardware with the real sensor.

- Extrinsic calibration routines depend on the [industrial_calibration](https://github.com/ros-industrial/industrial_calibration) package. Clone this package to your workspace.

- The industrial calibration library builds against `libceres`, an optimization library, whose installation instructions are available [here](http://ceres-solver.org/building.html).
