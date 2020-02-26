package frc.robot;

import frc.robot.util.geometry.Pose2d;
import frc.robot.util.geometry.Rotation2d;
import frc.robot.util.geometry.Translation2d;

// Any Sort of Constant or 'Magic Number' should be defined here
public class Constants {

  // Controller Ports
  public static final int DriverControllerPort = 0;
  public static final String DriverControllerName = "Controller (Xbox One For Windows)";
  public static final int OperatorControllerPort = 1;
  public static final String OperatorControllerName = "AIRFLO";

  // PWM
  public static final int kCameraRingId = 0;

  // Drive Constants
  public static final double kJoystickThreshold = 0.15;
  // Oracle's official constant convention. Don't @ me.
  public static final double TICKS_PER_FOOT = 1228.615;

  // Auto Constants
  public static final int AUTO_DEBOUNCE_TICKS = 10; // ~0.2 seconds

  // 22 is too low, 100 is too high
  public static final double kDriveWheelTrackWidthInches = 50;

  public static final double kDriveWheelDiameterInches = 3.938;
  public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
  public static final double kDriveWheelTrackRadiusWidthMeters =
      kDriveWheelTrackWidthInches / 2.0 * 0.0254;

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

  // Camera Specific Information
  public static final double kCameraDiagonalView = Math.toRadians(75);
  public static final double kCameraHorizontalAspect = 4;
  public static final double kCameraVerticalAspect = 3;
  public static final double kDiagonalAspect =
      Math.hypot(kCameraHorizontalAspect, kCameraVerticalAspect);
  public static final double kCameraHorizontalView =
      Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraHorizontalAspect / kCameraDiagonalView))
          * 2;
  public static final double kCameraVerticalView =
      Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraVerticalAspect / kCameraDiagonalView))
          * 2;
  public static final Rotation2d kShooterCameraHorizontalPlaneToLens =
      Rotation2d.fromDegrees(0); // Shooter should sit pretty flat
  public static final Rotation2d kBallCameraHorizontalPlaneToLens =
      Rotation2d.fromDegrees(-5); // camera is angled downards
  public static final double kShooterCameraHeight = 40; // shooter camera height on robot (inches)
  public static final double kBallCameraHeight = 12; // ball camera height

  // Offsets from our center point
  public static final Pose2d kTurrentToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));
  public static final Pose2d kWheelsToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));

  // Field Related Constants
  public static final double kHighGoalHeight = 96.25; // Center Goal Height
  public static final double kBallHeight = 5; // ball height (inches)

  // Looper System
  // period at which the looper runs at
  public static final double kLooperDt = 0.01;

  // Constants for Server Motor System
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates
  public static final int kLongCANTimeoutMs = 100; // use for constructor

  // Turret
  public static final int kTurretTalonMotorPort = 20;
  public static final double kTurretAimAngleDeadband = 1.5;
  public static final double kTicksPerDegee = 140;
}
