package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drive;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.geometry.*;
import frc.robot.vision.TargetInfo;
import frc.robot.vision.GoalTracker;
import java.util.*;

/*
    Robot State handles robot tra king throughout the match
    Handles all rotation positional information


    This code uses the concept of Poses2D, which essentially boils down to an x, y and rotation
    for example
        when we capture a target:
            Detected Goal Pose -> X_Goal, Y_Goal, Theta_Goal
            Robot Pose at Capture Time -> X_Old, Y_Old, Theta_Old

        However our robot pose is now likely different. We can compensate for this!
        Our Robot Pose now is -> X_Now, Y_Now, Theta_Now

        To find our Angle to Aim -> Atan2(Y_Goal - Y_Now, X_Goal - X_Now)
        To find our Range -> Sqrt((Y_Goal - Y_now)^2  + (X_Goal - X_now)^2 )



*/
public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    // Class Variables
    private double DistanceDriven; // Distance the robot drives

    // InterpolatingTreeMaps allow getting points that don't exist
    // Uses interopulating maps to use linear interopulation
    // Uses a max queue size passed in upon construction
    // Field to Vehicle Map is a Pose because there is X,Y point on the field, and a robot rotation
    // Vehicle to Turret map is only a roation in the 2D space as we don't expect the turret to
    // leave the robot
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> Field_To_Vehicle_Map;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> Vehicle_To_Turret_Map;

    // Size of the Storage Buffers
    // We don't want to carry TOO many values and overuse ram
    private static final int kObservationBufferSize = 100;

    private Twist2d vehicle_velocity_predicted;
    private Twist2d vehicle_velocity_measured;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered;

    // Reference to Drive Train
    // Used for polling encoders/gyro
    private Drive mDrive = Drive.getInstance();

    //Goal Trackers - Specific Goal Tracking Objects
    private GoalTracker vision_highgoal = new GoalTracker();
    private GoalTracker vision_ball = new GoalTracker();

    private List<Translation2d> mCameraToHighGoal = new ArrayList<>();
    private List<Translation2d> mCameraToBall = new ArrayList<>();

    // Variables Related to the Update
    // These variables are the back
    private double update_left_encoder_prev_distance = 0.0;
    private double update_right_encoder_prev_distance = 0.0;
    private double update_prev_timestamp = -1.0;
    private Rotation2d update_prev_heading = null;

    // Constructor for Robot State
    // Called upon RobotState startup, reset everything
    private RobotState() {
        // At the time of this call, this is our zero point!
        zero(0.0, Pose2d.identity(), Rotation2d.identity());
    }

    //Zero out the robot's state
    //when this called this is our new starting point
    //The Pose2D will be the initial x, y of the robot.. along with the rotation (which is all 0 since this is our starting point)
    //The Rotation will be the Turret's rotation relative to the robot (which is 0 since we are 'zeroing')
    public synchronized void zero(double time, Pose2d Initial_Robot_Pose, Rotation2d Initial_Turret_Rotation){
        //We haven't moved!
        DistanceDriven = 0;

        //Create our TreeMaps. These allow time based lookups
        //buffer will only hold a certain amount of points
        Field_To_Vehicle_Map = new InterpolatingTreeMap<>(kObservationBufferSize);
        Vehicle_To_Turret_Map = new InterpolatingTreeMap<>(kObservationBufferSize);

        //Store Intial Values
        Field_To_Vehicle_Map.put(new InterpolatingDouble(time), Initial_Robot_Pose);
        Vehicle_To_Turret_Map.put(new InterpolatingDouble(time), Initial_Turret_Rotation);

        //Velocity predicition values
        vehicle_velocity_measured = Twist2d.identity();
        vehicle_velocity_predicted = Twist2d.identity();
        vehicle_velocity_measured_filtered = new MovingAverageTwist2d(25); //maximum of 25 values
    }

    //Zero out everything...
    public synchronized void reset(){
        zero(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
    }


    // Update Robot State
    public void update(double timestamp) {
        // Check if previous heading has been set already
        if (update_prev_heading == null) {
            update_prev_heading = getLatestFieldToVehicle().getValue().getRotation();
        }

        final double dt = timestamp - update_prev_timestamp;
        final double left_distance = mDrive.getLeftEncoderDistance();
        final double right_distance = mDrive.getRightEncoderDistance();
        final double delta_left = left_distance - update_left_encoder_prev_distance;
        final double delta_right = right_distance - update_right_encoder_prev_distance;
        final Rotation2d gryo_angle = mDrive.getRotation();

        Twist2d odometry_twist;
        synchronized (this) {
            final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
            odometry_twist =
                    Kinematics.forwardKinematics(
                            last_measurement.getRotation(), delta_left, delta_right, gryo_angle);
        }

        final Twist2d measured_velocity =
                Kinematics.forwardKinematics(
                                delta_left,
                                delta_right,
                                update_prev_heading.inverse().rotateBy(gryo_angle).getRadians())
                        .scaled(1.0 / dt);
        final Twist2d predicted_velocity =
                Kinematics.forwardKinematics(
                                mDrive.getLeftLinearVelocity(), mDrive.getRightLinearVelocity())
                        .scaled(dt);

        // Update the Robot State Datastructures with new measures
        /*
        addVehicleToTurretObservation(timestamp,
            Rotation2d.fromDegrees(angle_degrees)
        )
        */

        // Update loops for the next cycle
        update_left_encoder_prev_distance = left_distance;
        update_right_encoder_prev_distance = right_distance;
        update_prev_heading = gryo_angle;
        update_prev_timestamp = timestamp;
    }



    public synchronized void ResetDriveDistance() {
        DistanceDriven = 0;
    }


    // Vision Manager Passes a new Target Info to the Robot State
    public synchronized void AddVisionObservation(TargetInfo ti) {
        double timestamp = Timer.getFPGATimestamp();
        Translation2d translation;

        // Proceed based on target type
        if (ti.IsHighGoal() == true) {
            // High Goal!
            translation = getCameraToVisionTargetPose(true, ti);
        } else {
            // Ball
            translation = getCameraToVisionTargetPose(false, ti);

        }
    }

    //Returns a translation of the robot 
    //returns null if there is no intersection with the goal.. no shot!
    private Translation2d getCameraToVisionTargetPose(boolean TurretCamera, TargetInfo ti){

        //Compensate for camera pitch!
        Translation2d xz_plane_translation = new Translation2d(ti.getX(), ti.getZ()).rotateBy(Rotation2d.identity());

        double x = xz_plane_translation.x();
        double y = ti.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        //if we do have an intersection with the goal, return our translation.. if we don't return null
        //double differential_height = source.getLensHeight() - (high ? Constants.kPortTargetHeight : Constants.kHatchTargetHeight);
        double differential_height = Constants.kHighGoalHeight;
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }

    /**
     * Return the Robot's Position on the field at a certain time. Linearly interpolates between
     * shared frames to fill in gaps
     *
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return Field_To_Vehicle_Map.getInterpolated(new InterpolatingDouble(timestamp));
    }

    /**
     * Return the Robot's Turrets Rotation
     *
     * @param timestamp
     * @return
     */
    public synchronized Rotation2d getVehicleToTurret(double timestamp) {
        return Vehicle_To_Turret_Map.getInterpolated(new InterpolatingDouble(timestamp));
    }

    /**
     * Get Turrets Pose on the Field This will be basically be the robots x,y and then the rotation
     * of the turret Obivously the turret's x,y is just the robots x,y turrets rotation can be
     * different from robot
     *
     * @return
     */
    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp)
                .transformBy(Pose2d.fromRotation(getVehicleToTurret(timestamp)));
    }

    // Get Latest Vehicle's Pose on the Field
    // Gets most up to date entry
    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return Field_To_Vehicle_Map.lastEntry();
    }


    // 
    public synchronized void addObservations(
            double timestamp,
            Twist2d displacement,
            Twist2d measured_velocity,
            Twist2d predicted_velocity) {

    }
}
