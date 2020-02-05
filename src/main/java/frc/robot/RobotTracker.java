package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drive;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.geometry.*;
import frc.robot.vision.GoalTracker;
import frc.robot.vision.TargetInfo;
import java.util.*;


/*
RobotTracker (formly RobotState) keeps track of the poses of various coordinate frames throughout the match.
Coordinate frame is a point (x,y) and a direction.

RobotTracker has update messages called from 

RobotTracker also interfaces with the vision system, and calculates what our turret needs to turn by
from Robot we can the then call:
getVisionError


Field-To-Vehicle -
Measurement of where the robot is on the field. There is inevitable drift, but is typically accurate over short time periods
Vehicle-To-Turret -
Measurement of where the turrets rotation is relative to the vehicle
*/

public class RobotTracker{
    private static RobotTracker mInstance;

    public static RobotTracker getInstance(){
        if(mInstance == null){
            mInstance = new RobotTracker();
        }
        return mInstance;
    }

    // Size of the Storage Buffers
  // We don't want to carry TOO many values
  private static final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> mField_to_Robot; //Robot's Pose on the Field
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> mRobot_to_Turret; //Turret's Rotation

    //Robot Drive Velocity Predictors
    private Twist2d mRobot_velocity_predicted;
    private Twist2d mRobot_velocity_measured;
    private MovingAverageTwist2d mRobot_velocity_measured_filtered;

    //Tracker for how far the robot has driven
    private double mRobot_Distance_Driven = 0;

    //Goal Trackers
    //Each vision target is a goal
    //so goal trackers for balls, and high goal
    private GoalTracker mVisionTarget_Ball;
    private GoalTracker mVisionTarget_Goal;

    //Lists of Translations to the Vision Targets
    List<Translation2d> mCameraToVisionTarget_Ball = new ArrayList<>();
    List<Translation2d> mCameraToVisionTarget_Goal = new ArrayList<>();

    //Reset the Robot. This is our zero point!
    private RobotTracker(){
        //Resets are called with everything at 0
        reset(0.0, Pose2d.identity(), Rotation2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     * this is the robot's position zero point!
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
                                   Rotation2d initial_vehicle_to_turret) {

        //Call Drive Reset
        reset(start_time, initial_field_to_vehicle);

        //Turret Related Reset
        //Create new map and store inital value
        mRobot_to_Turret = new InterpolatingTreeMap<>(kObservationBufferSize);
        mRobot_to_Turret.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    //Reset specific to the drive system
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        //Interpolating map related to robot's position on the field
        mField_to_Robot = new InterpolatingTreeMap<>(kObservationBufferSize);
        mField_to_Robot.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);

        //Setup Robot velcoity predictors
        mRobot_velocity_predicted = Twist2d.identity();
        mRobot_velocity_measured = Twist2d.identity();
        mRobot_velocity_measured_filtered = new MovingAverageTwist2d(25);

        //Zero out our drive distance tracker
        mRobot_Distance_Driven = 0.0;
    }


    //Reset
    //intended to be use
    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
    }


}