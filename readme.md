# Entropy 2020

This is a baseline of the 2020 Robotics Code. This code seeks to provide the team with a strong baseline entering the 2020 Build season.
This readme highlights portions of the architecture, ideas, and best practices.

### Code Formatting
We have a git hook that will format your code according to Google's style guide whenever you commit. To enable it, you will need to run
`update-hooks.bat` on Windows or `gradlew installGitHooks` on Linux and macOS.

### Architecture Highlights
- Multithreading Support
    - RoboRio has a 2 Core Processor and supports paralization. A good example of this is utilizing a secondary thread for vision communication.
- Field Relative Controls
    - We may find that subsystems would be better suited with field based controls. 
        - Turret for example, Don't force the drivers to have to overthink where the turret will be
- Singleton Class approach
    - For many of the classes, like the subsytems, force the system to use a singelton method. Guard nesscary methods with a synchronized keyword.
- Backplane Approach
    - The general contept is to expose as much logic as possible to the Robot.java class. For example the drive operation is called from Robot.java.
    - The concept behind this is easier thinking at a systems level.
    - Drive as much logic in the `RobotLoop` function found in the Robot.java.  
- Subsystem Subclass and Subsystem Manager
    - All subsystems must extend the abstract subsystem class
    - This adds subsystems automatically to the constructor via the abstract class
    - Intialially all this really is used for is to quickly loop through all the subsystems and run a subsystem check.
- Subsystem Checks
    - All Subsystem classes must extend the Subsystem class. Within that class is a Subsystem Check Class. 
    - If we intend on playing deep into playoff matchups, it is likely we will encounter a match with a fast turnaround to the next match. It would be good to have a fast and effective functional that could give us some confidence.
- RobotState
    - Essentially just a position Tracker for the robot. Record Pose2D at points.
- Disabled States
    - Likely not super useful, but we may find the need to perform periodic actions in a disabled state.
- Path Planning
    - Found in the Utility Libaries, we inherit 254's path planning logic. Using this logic we can drive waypoints. This library does an excellent job at pushing the robot's capabilites through spines while maintaining accuracy.

### Brainstorming Ideas
- 'Playbook'
    - Game permitting, we might find there are tasks that we will need to repeat often. For example, say if we are picking up a scoring object at a feeder station and want to make an immediate drive to a scoring station. A simple look at this would be:
        - Load Game Object -> Driver triggers playbook -> Robot performs autonomous drive to location -> Trigger Camera or other sensor Feedback -> Score Object
        - Directional Pad on controller could be a good control to do this.
- Preset Control
    - All Manipulators should have a 'state' based focus. With a state based system we should rely on PID Control to reach exact points
- SmartNav
    

### Concepts
- Physics
    We adopt some physic concepts that teams like 254 used (which are now included in the WPILIB apparently). Here is a brief explanation of each.
    - Kinematics
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
    - Translation2D
        - Translation in X and Y. Often times this is essentially used as an X, Y point for the robot on the field. PathSegment.java's constructor shows how this is done.
    - Rotation2D
        - Simply a point on the unit circle (cosine and sine). Basically represents a rotation of the robot. You can think of this like the Robot is in the center of a Unit Circle, with values plugged into Cos, Sin functions to pinpoint the robots rotation.
    - Coordinate Frame
        - X,Y point on a field, and a direction. In this case a Translation2D object would represent an X,Y position, and a Rotation2D object is the rotation. We will take a Coordinate frame upon robot intialization.

- PID
    - Proportional, Integral, Derivative 
- Motion Profiles
    - Jerk, Acceleration, Velocity, Position
    - Figure out where you want to go
    - Find a path
    - FInd a trajectory
    - Follow the trajectory
        - Figure out where you should be right now
        - Feedforware control + Feedback control




### Simulator
WPILib has simulator capability. We should utilize this simulator to simulate controller input early/motor output on the build season to iron out any glaring issues.
https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/introduction.html




### Code Housekeeping
We want our code to execute as efficiently as possible. 
- Make use of primitives types. Use int and double instead of Integer and Double. The JVM is able to store primitive types in the stack instead of the heap.
- Avoid using the "+" operator to concatenate strings *when you need to concatenate the result again* in a *different statement.*
    - See [This StackOverflow Answer](https://stackoverflow.com/a/4649160)
    - TL;DR: The compiler will translate concatenations to StringBuilder operations, but in some cases this may not be optimal. 
- Use a leading 'm' on instance variables `int mInstanceVariable = 0;`
- Use a leading 's' on static ("class") variables `int sClassVariable = 0;`
- Use a leading k on Constants `kDriveSpeed`
- Define all constants in Constants.java, to avoid reusing memory on the stack as needed


