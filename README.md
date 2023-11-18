# Agri Navigation ROS1

### build
```bash
cd ~/catkin_ws/src
git clone git@github.com:AgriSwarm/agri_navigation_ros1.git
# 依存関係があるのでビルドの順番を気をつける
catkin build quadrotor_msgs
catkin build pose_utils
catkin build
```
### demo
```bash
# first terminal
catkin source
roslaunch ego_planner rviz.launch
# second terminal
catkin source
roslaunch ego_planner single_drone_interactive.launch
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