# Clawed Flapper Introduction

## Tutorials worth to follow
Please refer to the tutorials in 
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html

Learn necessary ros2 tutorials also:
https://docs.ros.org/en/humble/index.html

## Folder sturcture
```
clawed_flapper
├── clawed_flapper
│   ├── my_clawed_flapper.py
│   └── reduced_att_controller.py
├── flapper_instruction.odt
├── flapper_messages.zip
├── launch
│   └── clawed_flapper_launch.py
├── lib
│   ├── ArrowManager.py
│   ├── Bladelement.py
│   ├── ClawTest.wbt
│   ├── FilterLib.py
│   ├── Flap3DOrgtest.wbt
│   ├── FTreducedAttCon.py
│   ├── FTslideControl.py
│   ├── ObserverLib.py
│   ├── readdata4text.py
│   ├── README.md
│   ├── RotationComputation.py
│   ├── SpecialorthogonalControl.py
│   └── testCount.py
├── LICENSE
├── package.xml
├── README.md
├── resource
│   ├── clawed_flapper
│   ├── clawed_flapper.urdf
│   ├── SimpleFlapper0Tail_H20BLE_X_pos.npy
│   ├── SimpleFlapper0Tail_H20BLE_Y_pos.npy
│   ├── SimpleFlapper0Tail_V20BLE_X_pos.npy
│   ├── SimpleFlapper0Tail_V20BLE_Y_pos.npy
│   ├── SimpleFlapper0Wing40BLE_X_pos.npy
│   └── SimpleFlapper0Wing40BLE_Y_pos.npy
├── setup.cfg
├── setup.py
└── worlds
    └── clawed_flapper_in_ros2.wbt
```

## Entrances
The entrances are `reduced_att_controller` and `clawed_flapper`.

See  following code in `setup.py`
```python
entry_points={
        'console_scripts': [
            'my_clawed_flapper = clawed_flapper.my_clawed_flapper:main',
            'reduced_att_controller = clawed_flapper.reduced_att_controller:main'
        ],
```

And we can see the following lines in the launch file `clawed_flapper_launch.py`: 
```python
clawed_flapper_ = WebotsController(
        robot_name='ClawedFlapper',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    
    reduced_att_controller_= Node(
        package='clawed_flapper',
        executable='reduced_att_controller',
    )
    return LaunchDescription([
        webots,
        # webots._supervisor,
        clawed_flapper_,
        reduced_att_controller_,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```
where we defined the webot controller and a node.

## clawed_flapper_
`clawed_flapper_` is what is used as the controller in webots.

In Webots, a controller is a program that controls a robot or simulation entity (like a drone, arm, or car) during the simulation. It defines the logic that the robot follows — like navigating, responding to sensors, or executing tasks.

`clawed_flapper_`

-->lib.Bladelement

-->lib.RotationComputation 

-->lib.FilterLib

Only three file are used.

The most important computation is in `Bladelement`,
which computes the aerodynamics in the quasi-steady blade-element way.

In this file, the following codes are given:
```python
 self. angular_vel_publisher         = self.__node . create_publisher(AngularVel, 'angular_vel_topic', 10)
        self. reduced_att_gamma_publisher   = self.__node . create_publisher(UnderactuatedAttGamma, 'un_att_gamma_topic', 10)
        self. translation_publisher         = self.__node . create_publisher(Translation, 'translation_topic', 10)
        
        self. reduced_com_subscriber        = self.__node . create_subscription(UnderactuatedCom, 'un_att_com', self.com_callback, 1)
```
This lines define the subscribers the topic for the control inputs. 

## reduced_att_controller
The `reduced_att_controller` is responsible for managing the reduced attitude control of the Clawed Flapper. Reduced attitude control refers to controlling the orientation of the robot with respect to a reduced set of rotational degrees of freedom, which can simplify the control problem. This controller is likely involved in maintaining a specific orientation or trajectory of the robot's body while it operates within the simulation environment.

In the context of the `clawed_flapper`, this controller would subscribe to necessary topics, process sensor data, and compute the control signals required to achieve the desired attitude. 
The codes are as follow: 
```python
   # controller command publisher
        self. com_publisher = self.create_publisher(UnderactuatedCom, 'un_att_com', 1)
        self. peaker_publisher = self.create_publisher(Float64, 'peaker', 1)
        
        
        self.create_subscription(AngularVel, 'angular_vel_topic', self.angular_vel_read_callback, 1)
        self.create_subscription(UnderactuatedAttGamma, 'un_att_gamma_topic', self.underactuated_att_gamma_read_callback, 1)
        self.create_subscription(Translation, 'translation_topic', self.translation_read_callback, 1)
        
```

It interacts with the ROS 2 ecosystem, utilizing nodes, publishers, and subscribers to seamlessly integrate with other components of the robotic system, ensuring that the Clawed Flapper maintains its intended orientation during flight or other operations.

The `reduced_att_controller` is launched as a ROS 2 node, allowing it to perform its control tasks in real-time, responding to changes in the robot's state and adapting its control outputs accordingly. This controller leverages specific libraries and algorithms to compute the necessary control actions, ensuring stability and performance of the robotic system under various conditions.

The flapper_messages are defined externally, but are used in `reduced_att_controller`

# Summary
In summary, the `clawed_flapper` package provides two ROS 2 nodes that collaborate to control the Clawed Flapper robot in a simulation environment. The `my_clawed_flapper` node serves as a bridge between the Webots simulator and the ROS 2 ecosystem, while the `reduced_att_controller` node is responsible for managing the reduced attitude control of the robot.
