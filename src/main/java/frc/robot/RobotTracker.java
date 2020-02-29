package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.Units;
import frc.robot.util.geometry.*;
import frc.robot.vision.AimingParameters;
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

public class RobotTracker {
  private static RobotTracker mInstance;

  public static RobotTracker getInstance() {
    if (mInstance == null) {
      mInstance = new RobotTracker();
    }
    return mInstance;
  }

  // Result Class
  // Used to return values from the Robot Tracker
  // Particularly with Vision Tracking
  // as well as target distance
  public class RobotTrackerResult {
    public final double tangental_component;
    public final double angular_component;
    public final Rotation2d turret_error;
    public final double distance;
    public final boolean HasResult;

    public RobotTrackerResult(
        double t_tangental_component,
        double t_angular_component,
        Rotation2d t_turret_error,
        double t_distance) {
      this.tangental_component = t_tangental_component;
      this.angular_component = t_angular_component;
      this.turret_error = t_turret_error;
      this.distance = t_distance;
      this.HasResult = true;
    }

    // Empty constructor!
    // Resultless Robot Tracker Object
    public RobotTrackerResult() {
      this.HasResult = false;
      this.tangental_component = 0;
      this.angular_component = 0;
      this.turret_error = null;
      this.distance = 0;
    }
  }

    //all these data structures are guarded by locks for improved synchronization


    // Size of the Storage Buffers
    // We don't want to carry TOO many values
    private final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> mField_to_Robot; //Robot's Pose on the Field
    private final Object mField_to_Robot_Lock = new Object();

    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> mRobot_to_Turret; //Turret's Rotation
    private final Object mRobot_to_Turret_Lock = new Object();

    //Robot Drive Velocity Predictors
    private Twist2d mRobot_velocity_predicted;
    private final Object mRobot_velocity_predicted_Lock = new Object();

    private Twist2d mRobot_velocity_measured;
    private final Object mRobot_velocity_measured_Lock = new Object();

    private MovingAverageTwist2d mRobot_velocity_measured_filtered;
    private final Object mRobot_velocity_measured_filtered_Lock = new Object();

    //Tracker for how far the robot has driven
    private double mRobot_Distance_Driven = 0;

    //Goal Trackers
    //Each vision target is a goal
    //so goal trackers for balls, and high goal
    private GoalTracker mVisionTarget_Ball = new GoalTracker(1);
    private final Object mVisionTarget_Ball_Lock = new Object();


    private GoalTracker mVisionTarget_Goal = new GoalTracker(2);
    private final Object mVisionTarget_Goal_Lock = new Object();

    //Lists of Translations to the Vision Targets
    List<Translation2d> mCameraToVisionTarget_Ball = new ArrayList<>();
    private final Object mCameraToVisionTarget_Ball_Lock = new Object();


    List<Translation2d> mCameraToVisionTarget_Goal = new ArrayList<>();
    private final Object mCameraToVisionTarget_Goal_Lock = new Object();

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

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToRobot(double timestamp) {
        return mField_to_Robot.getInterpolated(new InterpolatingDouble(timestamp));
    }

    //Same thing, more intuitive method
    public synchronized Pose2d getRobotPosition(double timestamp){
        return getFieldToRobot(timestamp);
    }

    // Get Turrets Rotation
    public synchronized Rotation2d getRobotToTurret(double timestamp) {
        return mRobot_to_Turret.getInterpolated(new InterpolatingDouble(timestamp));
    }

    //Get Turrets Rotation
    public synchronized Rotation2d getTurretRotation(double timestamp){
        return getRobotToTurret(timestamp);
    }


    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToRobot(timestamp).transformBy(Pose2d.fromRotation(getRobotToTurret(timestamp)));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToRobot() {
        return mField_to_Robot.lastEntry();
    }
    mRobot_velocity_predicted = predicted_velocity;
  }

  // Encoder Based Distance Driven
  public synchronized double getDistanceDriven() {
    return mRobot_Distance_Driven;
  }

  // Reset Distance driven
  public synchronized void resetDistanceDriven() {
    mRobot_Distance_Driven = 0.0;
  }

  // Robot's predicted velocity
  public synchronized Twist2d getPredictedVelocity() {
    return mRobot_velocity_predicted;
  }

  // Get Robot's measured velocity
  public synchronized Twist2d getMeasuredVelocity() {
    return mRobot_velocity_measured;
  }

  // based on moving average, get a smoothed average velocity
  public synchronized Twist2d getSmoothedVelocity() {
    return mRobot_velocity_measured_filtered.getAverage();
  }

  // reset vision targets
  public synchronized void resetVision() {
    mVisionTarget_Ball.reset();
    mVisionTarget_Goal.reset();
  }

  // Get translation
  /*

  */
  private Translation2d getCameraToVisionTargetPose(TargetInfo target, boolean highgoal) {
    Rotation2d SelectedCameraRotation;
    double TargetHeight;
    double LensHeight;
    // Select Rotation based on camera mount point
    if (highgoal) {
      // High Goal
      SelectedCameraRotation = Constants.kShooterCameraHorizontalPlaneToLens;
      TargetHeight = Constants.kHighGoalHeight;
      LensHeight = Constants.kShooterCameraHeight;
    } else {
      // Ball
      SelectedCameraRotation = Constants.kBallCameraHorizontalPlaneToLens;
      TargetHeight = Constants.kBallHeight;
      LensHeight = Constants.kBallCameraHeight;
    }

    // Compensate for camera pitch
    Translation2d xz_plane_translation =
        new Translation2d(target.getX(), target.getZ()).rotateBy(SelectedCameraRotation);
    double x = xz_plane_translation.x();
    double y = target.getY();
    double z = xz_plane_translation.y();

    //Store Pose2d of the robots position
    public synchronized void addFieldToRobotObservation(double timestamp, Pose2d observation) {
        mField_to_Robot.put(new InterpolatingDouble(timestamp), observation);
    }

    //Store information about turrets roation
    public synchronized void addRobotToTurretObservation(double timestamp, Rotation2d observation) {
        mRobot_to_Turret.put(new InterpolatingDouble(timestamp), observation);
    }

    //called from the robot tracker updater method
    //NOT USED.. for now...
    public synchronized void addDriveObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        mRobot_Distance_Driven += displacement.dx;
        addFieldToRobotObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToRobot().getValue(), displacement));

         mRobot_velocity_measured = measured_velocity;
        if (Math.abs(mRobot_velocity_measured.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            mRobot_velocity_measured_filtered.add(mRobot_velocity_measured);
        } else {
            mRobot_velocity_measured_filtered.add(new Twist2d(mRobot_velocity_measured.dx, mRobot_velocity_measured.dy, 0.0));
        }
        mRobot_velocity_predicted = predicted_velocity;
    }

    double distance = target.getDistance();
    distance = 3;
    Rotation2d angle = new Rotation2d(x, y, true);

    // System.out.println("Camera's Angle to Vision Target: " + angle.getDegrees());

    return new Translation2d(distance * angle.cos(), distance * angle.sin());

    /*
    if ((z < 0.0) == (differential_height > 0.0)) {
        double scaling = differential_height / -z;
        double distance = Math.hypot(x, y) * scaling;
        Rotation2d angle = new Rotation2d(x, y, true);
        return new Translation2d(distance * angle.cos(), distance * angle.sin());
    }
    */

    // return null;
  }

  // updates the goal tracker!
  // there is a goal tracker for
  private void updateGoalTracker(
      double timestamp, List<Translation2d> cameraToVisionTargetPose, boolean HighGoal) {
    GoalTracker SelectedTracker;
    Pose2d LensOffset; // Where from our center point this is mounted
    if (HighGoal == true) {
      // High Goal
      SelectedTracker = mVisionTarget_Goal;
      LensOffset = Constants.kTurrentToLens;
    } else {
      // Ball
      SelectedTracker = mVisionTarget_Ball;
      LensOffset = Constants.kWheelsToLens;
    }

    //reset vision targets
    //in a thread safe manner
    public void resetVision(){
        //reset ball tracker
        synchronized(mVisionTarget_Ball_Lock){
            mVisionTarget_Ball.reset();
        }

        //reset goal tracker
        synchronized(mVisionTarget_Goal_Lock){
            mVisionTarget_Goal.reset();
        }
    }

    /*
    if (cameraToVisionTargetPoses.size() != 2 ||
            cameraToVisionTargetPoses.get(0) == null ||
            cameraToVisionTargetPoses.get(1) == null) return;
    Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
            cameraToVisionTargetPoses.get(1), 0.5));
    */
    private Translation2d getCameraToVisionTargetPose(TargetInfo target, boolean highgoal) {
        Rotation2d SelectedCameraRotation;
        double TargetHeight;
        double LensHeight;
        //Select Rotation based on camera mount point
        if(highgoal){
            //High Goal 
            SelectedCameraRotation = Constants.kShooterCameraHorizontalPlaneToLens;
            TargetHeight = Constants.kHighGoalHeight;
            LensHeight = Constants.kShooterCameraHeight;
        }else{
            //Ball
            SelectedCameraRotation = Constants.kBallCameraHorizontalPlaneToLens;
            TargetHeight = Constants.kBallHeight;
            LensHeight = Constants.kBallCameraHeight;
        }
        

        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(SelectedCameraRotation);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        //254's distance method
        /*
        double differential_height = TargetHeight - LensHeight;
        double scaling = differential_height / z;
        scaling = 20;
        double distance = Math.hypot(x, y) * scaling;
        */

        double distance = target.getDistance();
        distance = 3;
        Rotation2d angle = new Rotation2d(x, y, true);

      // System.out.println("Camera's Angle to Vision Target: " + angle.getDegrees());
        Translation2d t = new Translation2d(distance * angle.cos(), distance * angle.sin());
        t.StoreDistance = target.getDistance();
        return t;

        /*
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        */

        //return null;
    }

    //updates the goal tracker!
    //there is a goal tracker for 
    private void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPose, boolean HighGoal) {
        Object SelectedLock;
        GoalTracker SelectedTracker;
        Pose2d LensOffset; //Where from our center point this is mounted
        if(HighGoal == true){
            //High Goal
            SelectedTracker = mVisionTarget_Goal;
            SelectedLock = mVisionTarget_Goal_Lock;
            LensOffset = Constants.kTurrentToLens;
        }else{
            //Ball
            SelectedTracker = mVisionTarget_Ball;
            SelectedLock = mVisionTarget_Ball_Lock;
            LensOffset = Constants.kWheelsToLens;
        }

        //Select Pose2d
        //254 performed an interpolation here, for now we will just use the one we have
        Translation2d SelectedTranslation = cameraToVisionTargetPose.get(0);
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(SelectedTranslation);

        /*
        if (cameraToVisionTargetPoses.size() != 2 ||
                cameraToVisionTargetPoses.get(0) == null ||
                cameraToVisionTargetPoses.get(1) == null) return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
                cameraToVisionTargetPoses.get(1), 0.5));
        */

        //Trasnlate from the vision points on the robot
        Pose2d fieldToVisionTarget = getFieldToTurret(timestamp).transformBy(LensOffset).transformBy(cameraToVisionTarget);


        //updated the selected tracker in a thread safe manner
        synchronized(SelectedLock){
            Pose2d NewPose = new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.identity());
            NewPose.StoredDistance = SelectedTranslation.StoreDistance;
            SelectedTracker.update(timestamp, List.of(NewPose));
        }
    }

    //Add vision packet
    //this takes the vision packet
    public void addVisionUpdate(double timestamp, TargetInfo observation) {
        boolean HighGoal = observation.IsHighGoal();

        //Perform Processing based on type of target
        if(HighGoal == true){
            //HighGoal
            synchronized(mCameraToVisionTarget_Goal_Lock){
                mCameraToVisionTarget_Goal.clear();
            }

            //This is built for multiple observations to stream in
            //right now we just stream in one
            if (observation == null) {
                synchronized(mVisionTarget_Goal_Lock){
                    mVisionTarget_Goal.update(timestamp, new ArrayList<>());
                }
                return;
            }

            //Get Camera to Target Pose
            synchronized(mCameraToVisionTarget_Goal_Lock){
                mCameraToVisionTarget_Goal.add(getCameraToVisionTargetPose(observation, true));
            }

            //Update Goal Tracker
            updateGoalTracker(timestamp, mCameraToVisionTarget_Goal, true);

        }else{
            //Ball
            synchronized(mCameraToVisionTarget_Ball_Lock){
                mCameraToVisionTarget_Ball.clear();
            }

            //This is built for multiple observations to stream in
            //right now we just stream in one
            if (observation == null) {
                synchronized(mVisionTarget_Ball_Lock){
                    mVisionTarget_Ball.update(timestamp, new ArrayList<>());
                }
                return;
            }

            //Get Camera to Target Pose
            synchronized(mCameraToVisionTarget_Ball_Lock){
                mCameraToVisionTarget_Ball.add(getCameraToVisionTargetPose(observation, false));
            }

            //Update Goal Tracker
            updateGoalTracker(timestamp, mCameraToVisionTarget_Ball, false);
        }
    }

    // return empty if nothing
    if (reports.isEmpty()) {
      // System.out.println("Returning Optional!");
      return Optional.empty();
    }

    //Return aiming information for the turret
    public Optional<AimingParameters> getAimingParameters(boolean highgoal, int prev_track_id, double max_track_age){
        AimingParameters params;
        GoalTracker SelectedTracker;
        Object SelectedLock;
        if(highgoal){
            //goal
            SelectedTracker = mVisionTarget_Goal;
            SelectedLock = mVisionTarget_Goal_Lock;
        }else{
            //ball
            SelectedTracker = mVisionTarget_Ball;
            SelectedLock = mVisionTarget_Ball_Lock;
        }

        synchronized(SelectedLock){
            List<GoalTracker.TrackReport> reports = SelectedTracker.getTrackReports();

            //return empty if nothing
            if(reports.isEmpty()){
                //System.out.println("Returning Optional!");
                return Optional.empty();
            }
    
            GoalTracker.TrackReport report = reports.get(0);
    
            double timestamp = Timer.getFPGATimestamp();
    
            //could perform sorting here for  multiple tracks
    
            //get pose of robot to goal
            Pose2d vehicleToGoal = getFieldToRobot(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());
    
            //Create Aiming Parameters output
            //includes stability score so we could decide if we wanted to use this
            params = new AimingParameters(
                report.id,
                report.latest_timestamp,
                report.stability,
                vehicleToGoal,
                report.field_to_target,
                report.field_to_target.getRotation()
            );
            params.SetVisionRange(report.distance);
        }

        return Optional.of(params);
    }


    public RobotTrackerResult GetFeederStationError(double timestamp){
        Optional<AimingParameters> mLatestAimingParameters = getAimingParameters(false, -1, Constants.kMaxGoalTrackAge);

        if(mLatestAimingParameters.isPresent()){
            //We have Aiming Parameters!

            //perform latency compensation
            //predfict robot's position
            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = getLatestFieldToRobot().getValue().inverse()
                    .transformBy(getPredictedFieldToVehicle(kLookaheadTime));

            //predicted robot to goal
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());

            double mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            //don't aim if not in distance range

            Rotation2d turret_error = getRobotToTurret(timestamp).getRotation().inverse().rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());


           // mLatestAimingParameters.get()

            Twist2d velocity = getMeasuredVelocity();
            RobotTrackerResult rtr = new RobotTrackerResult(
                mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange(),
                Units.radians_to_degrees(velocity.dtheta),
                turret_error,
                mLatestAimingParameters.get().GetVisionRange()
            );

            //System.out.println("REQ DEG: " + rtr.turret_error.getDegrees());

            return rtr;        
        }else{
            //We don't have aiming parameters!
            //don't move the turret!
            //empty object.. no results!
            RobotTrackerResult rtr = new RobotTrackerResult();
            return rtr; //0 rotation
        }

    }

    //Gets the turret error from the vision target
    //this is the function the turret will use to correct to
    public RobotTrackerResult GetTurretError(double timestamp){
        Optional<AimingParameters> mLatestAimingParameters = getAimingParameters(true, -1, Constants.kMaxGoalTrackAge);

        //check age here to make sure we didn't loose packets and this isn't really old

        if(mLatestAimingParameters.isPresent()){
            //We have Aiming Parameters!

            //perform latency compensation
            //predfict robot's position
            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = getLatestFieldToRobot().getValue().inverse()
                    .transformBy(getPredictedFieldToVehicle(kLookaheadTime));

            //predicted robot to goal
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());

            double mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            //don't aim if not in distance range

            Rotation2d turret_error = getRobotToTurret(timestamp).getRotation().inverse().rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            

            /*
           double t_tangental_component,
           double t_angular_component,
           Rotation2d t_turret_error,
           double t_distance,
           boolean t_HasResult

            */

            Twist2d velocity = getMeasuredVelocity();
            RobotTrackerResult rtr = new RobotTrackerResult(
                mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange(),
                Units.radians_to_degrees(velocity.dtheta),
                turret_error,
                mLatestAimingParameters.get().GetVisionRange()
            );

            //System.out.println("REQ DEG: " + rtr.turret_error.getDegrees());

            return rtr;

        }else{
            //We don't have aiming parameters!
            //don't move the turret!
            //empty object.. no results!
            RobotTrackerResult rtr = new RobotTrackerResult();
            return rtr; //0 rotation
        }
    }
  }

  public Pose2d getRobot() {
    return new Pose2d();
  }
}
