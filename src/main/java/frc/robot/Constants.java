package frc.robot;

import java.lang.Math;

import frc.robot.Config.Key;
import frc.robot.util.geometry.*;
import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
/*
  Constants
  Any thing used throughout the classes that dosen't need to be changed
  Or something that feels like a setting that we don't want to buy too much


  Interfaces to the config class (static constructor at bottom) to load in the config's
  parsed values
*/
public class Constants {
  // Controller Ports
  public static final int OperatorControllerPort = 1;
  public static final int DriverControllerPort = 0;

  // Drive Talon Variables
  public static final int kLeftDriveMasterId = 1;
  public static final int kLeftDriveSlaveId = 2;
  public static final int kRightDriveMasterId = 3;
  public static final int kRightDriveSlaveId = 4;

  // Drive Encoder Ports
  public static final int kLeftDriveEncoderPortA = 0;
  public static final int kLeftDriveEncoderPortB = 1;
  public static final int kRightDriveEncoderPortA = 2;
  public static final int kRightDriveEncoderPortB = 3;

  //Encoder Pulse Per Revolution PPR
  //number of high pulses an encoder will have on either of its square waves over a revolution
  public static final double kDriveEncoderPPR = 1000.0;


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
  public static final int kIntakeRollerPort = 10;
  public static final double kIntakeRollerSpeed = 0.8;

  //Turret
  public static final int kTurretTalonMotorPort;
  public static final double kPIDController_P;
  public static final double kPIDController_I;
  public static final double kPIDController_D;
  public static final int kTurretEncoderA = 4;
  public static final int kTurrentEncoderB = 5;


  //Storage
  public static final int kStorageLowerTalon = 7;
  public static final int kStorageUpperTalon = 8;

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
  public static final double kMaxGoalTrackAge = 5.0; //originally was 3, lets tune this down!
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
   public static final Rotation2d kBallCameraHorizontalPlaneToLens = Rotation2d.fromDegrees(-5); //camera is angled downards
   public static final double kShooterCameraHeight = 40; //shooter camera height on robot (inches)
   public static final double kBallCameraHeight = 12; //ball camera height
   
   //Offsets from our center point
   public static final Pose2d kTurrentToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)); 
   public static final Pose2d kWheelsToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)); 


  //Field Related Constants
  public static final double kHighGoalHeight = 96.25; //Center Goal Height
  public static final double kBallHeight = 5; //ball height (inches)

  //Looper System
  public static final double kLooperDt = 0.01;

  //Constants for Server Motor System
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates
  public static final int kLongCANTimeoutMs = 100; // use for constructors

  //turret constants
  // turret
  public static final ServoMotorSubsystemConstants kTurretConstants = new ServoMotorSubsystemConstants();
  static {
      kTurretConstants.kName = "Turret";

      kTurretConstants.kMasterConstants.id = 9;
      kTurretConstants.kMasterConstants.invert_motor = false;
      kTurretConstants.kMasterConstants.invert_sensor_phase = true;

      // Unit == Degrees
      kTurretConstants.kHomePosition = 0.0;  // CCW degrees from forward
      kTurretConstants.kTicksPerUnitDistance = 4096.0 * 72.0 / 18.0 * 54.0 / 16.0 / 360.0;
      kTurretConstants.kKp = 2.0;
      kTurretConstants.kKi = 0;
      kTurretConstants.kKd = 10.0;
      kTurretConstants.kKf = 0.08;
      kTurretConstants.kKa = 0.0;
      kTurretConstants.kMaxIntegralAccumulator = 0;
      kTurretConstants.kIZone = 0; // Ticks
      kTurretConstants.kDeadband = 0; // Ticks

      kTurretConstants.kPositionKp = 0.35;
      kTurretConstants.kPositionKi = 0.0;
      kTurretConstants.kPositionKd = 0.0;
      kTurretConstants.kPositionKf = 0.0;
      kTurretConstants.kPositionMaxIntegralAccumulator = 0;
      kTurretConstants.kPositionIZone = 0; // Ticks
      kTurretConstants.kPositionDeadband = 0; // Ticks

      kTurretConstants.kMinUnitsLimit = -135.0;
      kTurretConstants.kMaxUnitsLimit = 315.0;

      kTurretConstants.kCruiseVelocity = 5000; // Ticks / 100ms
      kTurretConstants.kAcceleration = 16000; // Ticks / 100ms / s
      kTurretConstants.kRampRate = 0.0; // s
      kTurretConstants.kContinuousCurrentLimit = 20; // amps
      kTurretConstants.kPeakCurrentLimit = 30; // amps
      kTurretConstants.kPeakCurrentDuration = 10; // milliseconds
      kTurretConstants.kMaxVoltage = 12.0;

     // kTurretConstants.kStatusFrame8UpdateRate = 50;
      kTurretConstants.kRecoverPositionOnReset = true;
  }



  //Constant Intialization!
  //Dyanmic Config System is loaded into true java constants
  //Doing this allows java to optimize around these values
  static {
    //Open Constants System and load into static variables
    Config mConfig = Config.getInstance();
    mConfig.reload();


    //Now our values are dynamically loaded in
    //lets now load these to our variables!
    
    //Intake
    //kIntakeRollerPort = mConfig.getInt(Key.INTAKE__ROLLER_PORT);

    //Turret
    kTurretTalonMotorPort = 7;
    kPIDController_P = mConfig.getDouble(Key.OI__VISION__PID__P);
    kPIDController_I = mConfig.getDouble(Key.OI__VISION__PID__I);
    kPIDController_D = mConfig.getDouble(Key.OI__VISION__PID__D);


  };
}
