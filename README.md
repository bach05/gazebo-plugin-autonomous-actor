# GAZEBO Plugin Autonomous Actor

This work is a further improvement of the AutonomousActorPlugin. It allows continuous wandering of an actor with obstacle avoidance in a personalized region.  

The package works under some assumption and the obstacle avoidance is not perfect yet. 

If you use this package for academic reasearch, please cite our work accepted for the 10th European Conference on Mobile Robots (ECMR 2021): 

```bash

@INPROCEEDINGS{bacchin-beraldo,

  author={Bacchin, Alberto and Beraldo, Gloria and Menegatti, Emanuele},

  booktitle={2021 European Conference on Mobile Robots (ECMR)}, 

  title={Learning to plan people-aware trajectories for robot navigation: A genetic algorithm}, 

  year={2021},
  
  volume={},

  number={},

  pages={},

  doi={}

}
```


## Installation

This code may be treated as a ROS package. Therefore, it can be built with `catkin_make` or `catkin build`.

```bash
cd your_workspace/src
git clone https://github.com/SharedSocialNavigation/gazebo-plugin-autonomous-actor.git
cd ..
catkin build
```

## Usage

Add your plugin in your sdf world file, under actor tag as in this example:  

```xml

<model name="actor0_collision_model">
  <pose>0 0 -100 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="link">
      <pose>0 -0.18 0.05 0 -1.5707963267948966 0</pose>
      <geometry>
        <box>
          <size>0.44 1.62 0.60</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>


<actor name="actor0">
    <pose>10 10 0 0 0 0</pose>
    <skin>
    <filename>walk.dae</filename>
    <scale>1.0</scale>
    </skin>
    <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.000000</scale>
    <interpolate_x>true</interpolate_x>
    </animation>

    <plugin name="actor0_plugin" filename="libAutonomousActorPlugin.so">
    <target_weight>1.15</target_weight>
    <obstacle_weight>1.8</obstacle_weight>
    <animation_factor>5.1</animation_factor>
  <!-- Usage: Initial pose of the actor -->
    <init_position>10 10 0</init_position>
    <!-- Usage: Modify the set of models that the vector field should
            ignore when moving the actor -->
    <ignore_obstacles>
        <model>ground_plane</model>
	<model>actor1_collision_model</model>
	<model>actor2_collision_model</model>
	<model>actor3_collision_model</model>
    </ignore_obstacles>
  <!-- Usage: Specify which other actors you want to avoid during wandering -->
    <actor_awareness>
        <model>actor1</model>
        <model>actor2</model>
        <model>actor3</model>
    </actor_awareness>
 <!-- Usage: Specify the first target -->
     <target>12 13 1.21</target>
<!-- Usage: Specify the region where the random targets are computed, it is recommended to stay 0.5 meters from walls -->
	<room_x>4.5 15.5</room_x>
	<room_y>3.5 15.5</room_y>
	<room_z>0</room_z>
    </plugin>
</actor>
```
**ATTENTION**: 
- you have to put your stuffs in the first quadrant of the world frame (e.i. x-y must be positive). This to avoid an unsolved bug. 
- To enable laser visible actors, I attached a collision box to each one using the AttachModelPlugin plugin from servicesim_competition. The laser now sees the collision box, but not the actor. A more detailed collision model should be designed using CollisionActorPlugin from servicesim_competition too, but currently it does not work properly. 

## Parameters

ACTOR  
- **pose**: Modify the <pose> element of each actor to change their starting location.

AUTONOMOUS ACTOR PLUGIN
- **target**: Add <target> element for actor to specify its first goal.
- **target_tolerance**: Modify <target_tolerance> for invoking next target. Please note that the Gazebo simulator is not very accurate.
- **init_position**: Starting position, should be the same as <pose>
- **ignore_obstacles**: Add <model> element that will be ingored by obstacle avoidance
- **actor_awareness**: Add <model> element of actor to be avoided (actors are treated in a different manner)
- **room_x/y/z**: Specify the region where the random targets are computed, it is recommended to stay 0.5 meters from walls


## Models

The models are obtained from [tiago_simulation](https://github.com/pal-robotics/tiago_simulation/tree/kinetic-devel/tiago_gazebo/models) and [servicesim_competition](https://github.com/osrf/servicesim/tree/master/servicesim_competition).

## More about Gazebo plugin

![image](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/install_dependencies_from_source/files/gazebo_dependency_tree.svg)

## Reference

- Reference plugin repo [link1](https://github.com/osrf/gazebo/tree/gazebo9/plugins)
- Gazebo debug msg [link1](https://answers.gazebosim.org//question/17428/how-print-the-output-of-a-plugin/)
- Gazebo dependencies [link1](http://gazebosim.org/tutorials?tut=install_dependencies_from_source)
- Gazebo actors [link1](http://gazebosim.org/tutorials?tut=actor&cat=build_robot)
- AutonomousActorPlugin [link1](https://github.com/BruceChanJianLe/gazebo-plugin-autonomous-actor)
- servicesim_competition [link1](https://github.com/osrf/servicesim/tree/master/servicesim_competition)
# gazebo-plugin-autonomous-actor
