# Agri Navigation ROS1

## Setup
```bash
cd ~/catkin_ws/src
git clone git@github.com:AgriSwarm/agri_navigation_ros1.git
git clone git@github.com:AgriSwarm/agri_resources.git
git clone git@github.com:AgriSwarm/realsense_gazebo_plugin.git
# 依存関係があるのでビルドの順番を気をつける
catkin build quadrotor_msgs
catkin build pose_utils
catkin build

echo "export GAZEBO_MODEL_PATH=/home/torobo/catkin_ws/src/agri_resources/models" >> ~/.bashrc
source ~/.bashrc
```

### build error
以下のbuildエラーが出る場合は
`/Utils/odom_visualization/CMakeLists.txt:167`の`pose_util`をビルドファイルのフルパスに書き換える

```
/usr/bin/ld: cannot find -lpose_utils
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/odom_visualization.dir/build.make:123: /home/torobo/catkin_ws/devel/.private/odom_visualization/lib/odom_visualization/odom_visualization] Error 1
make[1]: *** [CMakeFiles/Makefile2:1888: CMakeFiles/odom_visualization.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
```

## Ardupilot demo
```bash
# first terminal
sim_vehicle.py -m --streamrate=30 -v ArduCopter -f gazebo-iris  --console
# second terminal
roslaunch ego_planner mavros_demo.launch
```
**Wait until prearmed**

```bash
# first terminal(MAV console)
mode guided
arm throttle
takeoff 1
```

```bash
# third terminal
rostopic pub /goal_with_id quadrotor_msgs/GoalSet "drone_id: 0
goal: [0.0, 0.0, 3.0]" -1
```

## Pybullet demo
```bash
roslaunch ego_planner pybullet_demo.launch
```