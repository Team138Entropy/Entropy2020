package frc.robot;

import java.lang.Math;

import frc.robot.Config.Key;
import frc.robot.util.geometry.*;
/*
  Constants
  Any 


*/
public class Constants {
  // Controller Ports
  public static final int OperatorControllerPort = 1;
  public static final int DriverControllerPort = 0;

  // Talon Variables
  public static final int kLeftDriveMasterId = 1;
  public static final int kLeftDriveSlaveId = 2;
  public static final int kRightDriveMasterId = 3;
  public static final int kRightDriveSlaveId = 4;

  // Solenoid Constants
  public static final int kPCMId = 1;
  public static final int kShifterSolenoidId = 7;

  // PWM
  public static final int kCameraRingId = 0;

  // Drive Constants
  public static final double kJoystickThreshold = 0.2;
  public static final double kDriveWheelTrackWidthInches = 22.75;
  public static final double kDriveWheelDiameterInches = 3.938;
  public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
  public static final double kDriveWheelTrackRadiusWidthMeters =
      kDriveWheelTrackWidthInches / 2.0 * 0.0254;


  //Intake
  public static final int kIntakeRollerPort;

  //Storage
  //public static final int k

  //Drive Encoder Port
  public static final int kDriveGyroPort = 0;

  // Based on how this is used, I'm pretty sure this is a corrective factor
  public static final double kTrackScrubFactor = 1.0469745223;
  public static final double kDriveVoltageRampRate = 0.0;
  public static final int kDriveCurrentThrottledLimit = 30; // amps
  public static final int kDriveCurrentUnThrottledLimit = 80; // amps

  // Path Following Constants
  public static final double kPathFollowingMaxAccel = 80.0; // inches per second ^ 2

  // Vision Tracking Constants
  public static final double kMaxTrackerDistance = 60.0;
  public static final double kMaxGoalTrackAge = 3.0;
  public static final double kMaxGoalTrackAgeNotTracking = 0.1;
  public static final double kMaxGoalTrackSmoothingTime = 0.5;
  public static final double kTrackStabilityWeight = 0.0;
  public static final double kTrackAgeWeight = 10.0;
  public static final double kTrackSwitchingWeight = 100.0;
  public static final double kCameraFrameRate = 90.0; // fps

   //Camera Specific Information
   public static final double kCameraDiagonalView = Math.toRadians(75);
   public static final double kCameraHorizontalAspect = 4;
   public static final double kCameraVerticalAspect = 3;
   public static final double kDiagonalAspect = Math.hypot(kCameraHorizontalAspect, kCameraVerticalAspect);
   public static final double kCameraHorizontalView = Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraHorizontalAspect / kCameraDiagonalView)) * 2;
   public static final double kCameraVerticalView = Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraVerticalAspect / kCameraDiagonalView)) * 2;
   public static final Rotation2d kShooterCameraHorizontalPlaneToLens = Rotation2d.fromDegrees(0); //Shooter should sit pretty flat
   public static final Rotation2d kBallCameraHorizontalPlaneToLens = Rotation2d.fromDegrees(-15); //camera is angled downards
   public static final double kShooterCameraHeight = 40; //shooter camera height on robot (inches)
   public static final double kBallCameraHeight = 15; //ball camera height
   
   //Offsets from our center point
   public static final Pose2d kTurrentToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)); 
   public static final Pose2d kWheelsToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)); 


  //Field Related Constants
  public static final double kHighGoalHeight = 96.25; //Center Goal Height
  public static final double kBallHeight = 10; //ball height (inches)



  //Constant Intialization!
  //Dyanmic Config System is loaded into true java constants
  //Doing this allows java to optimize around these values
  static {
    //Open Constants System and load into static variables
    Config mConfig = Config.getInstance();
    mConfig.reload();


    //Now our values are dynamically loaded in
    //lets now load these to our variables!
    
    kIntakeRollerPort = mConfig.getInt(Key.INTAKE__ROLLER_PORT);

  };
}
