#Overview
To Navigate TurtleBot in the path following white ball.

# ROS_GO_chase_it_project
In this project we create two ROS packages inside your catkin_ws/src: the drive_bot and the ball_chaser which will be used in Gazebo for all our upcoming projects in the Udacity Robotics Software Engineer Nanodegree Program. Here are the steps to design the robot, house it inside our world, and program it to chase white-colored balls:

**drive_bot:**

- Create a my_robot ROS package to hold your robot, the white ball, and the world.
- Design a differential drive robot with the Unified Robot Description Format. Add two sensors to our robot: a lidar and a camera. Add Gazebo plugins for our robot’s differential drive, lidar, and camera. The robot we design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions.
- House our robot inside the world we built in the Build My World project.
- Add a white-colored ball to our Gazebo world and save a new copy of this world.
- The world.launch file should launch our world with the white-colored ball and our robot.

**ball_chaser:**
- Create a ball_chaser ROS package to hold our C++ nodes.
- Write a drive_bot C++ node that will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
- Write a process_image C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, our node should request a service via a client to drive the robot towards it.
- The ball_chaser.launch should run both the drive_bot and the process_image nodes.

# Project Description
```
Go-Chase-It                                   # Go Chase It Project
├── catkin_ws                                  # Catkin workspace
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files 
│   │   │   │   ├── myworld.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── pictures                                     
│   ├──                                         # images
```

# Run the project

1- Launch the robot inside your world
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
2- Run drive_bot and process_image
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
3- Visualize
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view  
```

# Image

![imge1](https://user-images.githubusercontent.com/49041896/90844858-dd5dbf80-e332-11ea-99b2-8187f6b296f0.png)
