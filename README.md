# Charged-Up-6722
This is the program for LED Robotics' robot, Banshee, for the 2023 FRC game, Charged Up.

This program is written in C++ using WPILib's Command-Based framework. 

Custom Subsystem objects have been made for each of the important mechanisms of the robot. 

Each Subsystem except the drive train acts like a state machine where it can be toggled between different control modes to do different things. 
Because of the design of this year's robot, each Subsystem is in a mode where their mechanisms follow positional or velocity PID loops. 

Every motor on the robot is a Falcon500, which means that all of them have built-in encoders. 
Our swerve modules have CTRE Mag Encoders for absolute positioning.

For performance and ease of tuning, all of the robot's mechanisms, including our swerve modules, use our Falcon500s built-in PID controllers. 
This simplifies our codebase significantly and increases the performance of our roboRIO.

Tele-op utilizes default Commands to handle driving, and intaking. All other controls for the robot are bound to Commands that handle manipulator positioning. 
The drivetrain can be toggled between field-centric and robot-oriented control schemes. 

Larger Commands for the robot are defined as their own files in the "commands" directories of the program, smaller Commands are defined in RobotContainer.h for ease of use.

Autonomous selection is done using a SendableChooser. Most autonomous routines are SequentialCommandGroups wrapped in a CommandPtr. Every autonomous routine is wrapped with a 15 second timeout to ensure that they stop after the autonomous period has elapsed. 
