# Entropy 2020

A baseline of the 2020 Robotics Code. This code seeks to provide the team with a strong baseline entering the 2020 Build season.
This readme highlights portions of the architecture, ideas, and best practices.

### Architecture Highlights
- Multithreading Support
    - RoboRio has a 2 Core Processor and supports paralization. A good example of this is utilizing a secondary thread for vision communication.
- Field Relative Controls
    - We may find that subsystems would be better suited with field based controls. 
        - Turret for example, Don't force the drivers to have to overthink where the turret will be
- Singleton Class approach
    - For many of the classes, like the subsytems, force the system to use a singelton method. Guard nesscary mmethods with a synchronized keywork.
- Backplane Approach
    - The general contept is to expose as much logic as possible to the Robot.java class. For example the drive operation is called from Robot.java.
    - The concept behind this is easier thinking at a systems level.
    - Drive as much logic in the `RobotLoop` function found in the Robot.java.  
- Subsystem Subclass and Subsystem Manager
    - All subsystems must inherit the abstract subsystem class
    - This adds subsystems automatically to the constructor via the abstract class
    - Intialially all this really is used for is to quickly loop through all the subsystems and run a subsystem check.
- Subsystem Checks
    - All Subsystem classes must inherit the Subsystem class. Within that class is a Subsystem Check Class. 
- RobotState
    - Essentially just a position Tracker for the robot. Record Pose2D at points.



### Concepts
- Physics
    We adopt some physic concepts that teams like 254 used (which are now included in the WPILIB apparently). Here is an explanation of each.
    -Kinematics
        - Branch of classical mechanics (physics of motion) that describes the motion of an object without knowing anything about the forces acting on the object.
        - Forward Kinematics
            - Convert positions of the robot to overal robot position
            - An example of this would be in a 2 jointed arm, forward kinematics would calculate the position of the end of the entire arm based on the angle of each joint
            - Robot Drive Example: Forward Kinematics determines position of drivebased based on position of the left and right sides of the drive base
        - Inverse Kinematics
            - Determines position of parts of the robot based on the robot position. The inverse of forward kinematics
            - On a 2 jointed arm, if we have a target location of where we want the end to be, we can calculate what angle each joint has to be to reach that position
            - Robot Drive Example: Determines left and right wheel speeds based on commanded velocities 
    - Pose2D (Rigid Transformation)
        - A transformation is when you take a shape and you move it in some way. A rigid transformation includes rotations, translations, and reflections. 
        - Rotation rotates the shape around a center point. Each rotation has a direction (clockwise or counter-clockwise), center point and degree of rotation
        - Translation is a sliding of a shape. An easy example is sliding a book 5 inches to the left on a bookshelf, this would be a translation. If we were zero'ed, on an x, y coordinate grid, then you could say you were moving the book along the x accesses. 
        - Reflections are pretty simply. Results in flipping the shape across some line. These aren't overly useful in our case. 
        - Each Pose2D contains a Translation2D Object and a Rotation2D Object. Essientally just a rotation vector and a position vector
    -Translation2D
        - Translation in X and Y. Often times this is essentially used as an X, Y point for the robot. PathSegment.java's constructor shows how this is done.
    -Rotation2D
        - Simply a point on the unit circle (cosine and sine). Basically represents a rotation of the robot.
    -Coordinate Frame
        - X,Y point on a field, and a direction. In this case, a 

- PID
    - Proportional, Integral, Derivative 
- Motion Profiles
    - Jerk, Acceleration, Velocity, Position
    - Figure out where you want to go
    - Find a path
    - FInd a trajectory
    - Follow the trajectory
        - FIgure out where you should be right now
        - Feedforware control + Feedback control
- Forward Kinematics
    - My left wheel went forward 2 inches, my right wheel went forward 4 inches. I went forward while turning counter-clockwise
- Inverse Kinematics
    - I want to turn clockwise 90 degrees. Left wheel must go forward for X inches, Right wheel must go backward for X inches

### Code Housekeeping
We want our code to execute as efficiently as possible. 
- Make use of primitives types. Use int and double instead of Integer and Double. The JVM is able to store primitive types in the stack instead of the heap.
- Use StringBuilder rather than + operator when combining strings


### Simulator
WPILib has simulator capability. We should utilize this simulator to simulate controller input
https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/introduction.html














