
# Welcome to Ros2FlapperSimu

## How to run the simulation:
```
ubuntu@ubuntu-ASUS-TUF-Gaming-F15-FX507ZV4-FX507ZV4:cd ros2_ws
ubuntu@ubuntu-ASUS-TUF-Gaming-F15-FX507ZV4-FX507ZV4:~/ros2_ws$ source install/local_setup.bash
ubuntu@ubuntu-ASUS-TUF-Gaming-F15-FX507ZV4-FX507ZV4:~/ros2_ws$ ros2 launch clawed_flapper clawed_flapper_launch.py
```

## How to build:
```
colcon build
```

To build a specific package with colcon, you can use the following command:

```
colcon build --packages-select <package_name>
```

Replace <package_name> with the name of the package you want to build.
For example, if the package you want to build is called my_robot_package, the command would be:

```
colcon build --packages-select my_robot_package
```

This will build only the specified package and its dependencies, rather than the entire workspace.
If you want to build multiple specific packages, you can list them:

```
colcon build --packages-select package1 package2 package3
```

Let me know if you need more help with colcon or ROS!
