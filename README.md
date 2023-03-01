[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=9535124&assignment_repo_type=AssignmentRepo)
# Final Project: Preliminary Navigation

## Background
Navigation is an essential functionality for any mobile robot. Autonomous navigation can be achieved with the help of all the sensors and actuators. In this project, we'll realize basic autonomous navigation with our robots.

## Procedure:
1. Set the robot behind the staring line.
2. Press the button to start navigation.
3. The robot should stop and terminate the navigation at the finish marker

## Requirements:
You are expected to
- Navigate the robot from the starting line to the marker out of the classroom. 
- Use at least one sensor from the following options:
    - Ultrasonic distance sensor
    - Lidar
    - Camera

**Note: DO NOT HARD CODE. Use at least one sensor. Or, 45% credits will not be given.**

### (50%) Robot Performance:
- (5%) Modes:
    - The robot has 3 modes: **WORK**, **PAUSE** and **OFF**. 
    - One LED will be used to indicate the robot's status ("always on" for **WORK**, "fade-in-fade-out" for **PAUSE**, "off" for **OFF**).
    - A button will be used to switch modes bewteen **WORK** and **PAUSE**.

- (10%) Checkpoint 1
    - The robot successfully passes the `Checkpoint 1` line starting from the `Start` line.
- (10%) Checkpoint 2
    - The robot successfully passes the `Checkpoint 2` line starting from the `Start` line.
- (10%) Checkpoint 3
    - The robot successfully passes the `Checkpoint 3` line starting from the `Start` line.
- (15%) Finish Marker
    - (5%) The robot successfully reached the `Finish` marker starting from the `Start` line.
    - (5%) The robot stopped at the `Finish` marker within 0.5 meters radius.
    - (5%) Turn off everything (except Raspberry Pi) on board.
    
**Note: you can guide your robot using ArUco marker after the robot passed Checkpoint 3.**
    

### (50%) Documentation
A well-documented project can help people who are interested to follow. Also, it will be helpful if you want to continue the work after a while.  
1. (20%) Methods
2. (5%) Part List.
3. (15%) Wiring Diagram.
4. (10%) Summary

## Methods
> This robot utilizes LIDAR to follow a wall on its left side. This is acomplished by checking three angles, forward (0 degrees), left (270 degrees), and forward-left (320 degrees). For each point, there is a distance threshold where values larger than the threshold return True and values smaller than the threshold return False. Depending on the combination of True and False readings, the robot performs one of a few functions. These are shown in the table below. This table also demonstrates the order that each condition is evaluated, there are some aparent redundancies in the logic, but they will never be able to evaluate at the same time due to an earlier if statement being true.
> 
| Forward | Left  | Forward_Left | Decision     |
|---------|-------|--------------|--------------|
| Any     | Any   | False        | Forward Left |
| True    | False | Any          | Forward Left |
| Any     | True  | False        | Forward Left |
| False   | False | Any          | Forward Left |
| True    | Any   | Any          | Right        |
| False   | True  | Any          | Forward      |
| Any     | Any   | Any          | Right        |

> Initially the bot only used the forward and left points for navigation, but it struggled to identify slight curvature in the wall. To combat this, the forward threshold value had to be increased to a point that caused unintended True flags. The forward-left sensor was added to combat this, but for a course with all right angle turns, it would probably be unecissary. Due to the sporatic nature of using the RPLidar with python, the program gets many false readings that lead to unintended behavior. In some cases, this can cause the bot to run into a wall. To combat this, another variable called e_forward (emergency forward) was added. If the bot runs into a wall and gets stuck, this variable goes true and triggers the bot to pivot away from the wall and resume driving. This beahvior continues until the camera on the front of the bot detects an AruCo marker in the view. At this point, the LIDAR code stops and the robot drives and steers based on the position of an AruCo marker in the frame. If the center of the marker crosses a certain distance from the center of the frame, the robot will turn to face the marker. Once the marker is within the boundry it drives forward. Below is a list of all of the APIs used for this project, as well as what each one way used for.

| API              | Utilization                                    |
|------------------|------------------------------------------------|
| adafruit_rplidar | Allows for reading of the RpLidar              |
| time             | Can sleep the program                          |
| gpiozero         | Motor driver code, led use, and button handler |
| picamera         | Allows for reading the PiCam                   |
| CV2              | Provides image processing and ArUco detection  |
| imutils.video    | Provides video streaming for debugging         |
| Math             | Gives angles for reading lidar datapoints      |


## Part List
| Component                    | Quantity | Description                                                     |
|------------------------------|----------|-----------------------------------------------------------------|
| Robot Base                   | 1        | 3D printed hexagonal base with caster mount                     |
| Robot 2nd Layer              | 1        | 3D printed hexagonal 2nd layer                                  |
| Raspberry Pi 4               | 1        | Microcontroller                                                 |
| Waveshare Motor Driver       | 1        | Motor controller board, GPIO header mounted                     |
| DC Motor (and mounts/wheels) | 2        | Drives the robot                                                |
| Caster Wheel                 | 1        | Rear wheel, unpowered, can freely roll in any direction         |
| Pi Cam                       | 1        | Raspberry Pi camera module, connects via Picam header on the Pi |
| Pi Cam Mount                 | 1        | 3D printed Pi Cam mount                                         |
| RpLidar                      | 1        | USB lidar                                                       |
| Battery Pack                 | 1        | Holds 2 batteries in series                                     |
| Battery                      | 2        | 3.8V LiPo                                                       |

## Wiring Diagram
> ![Circuit Diagram](https://github.com/UCAEngineeringPhysics/p2-preliminary_navigation-choggard123/blob/main/wall_follower_circuit.png)
> The LIDAR and camera were both connected using the Raspberry Pi's USB ports.

## Summary
> Overall, the robot was successful in navigating the course. There is some room for improvement and itteration in the future. The LIDAR will periodically fail to return accurate data when used in conjunction with the motor driver. This could be solved by trying different LIDAR modules, or using a software like ROS. The ArUco follower code could also be more smooth. Currently it has 3 states, forward, right, or left. Since the position of the marker is logged by the code, a steering algorythm based on the distance of the marker from the center could be made fairly easily. This would likely make it much smoother. The bot could also benifit from an improved base, the current one has a slight back angle which can cause the LIDAR to overlook the walls of the course.
> Link to video of the robot navigating the course: https://youtu.be/YXTO4WD8PSE
