package frc.robot;

// Any Sort of Constant or 'Magic Number' should be defined here
@Deprecated
public class Constants {
  // Controller Ports
  public static final int OperatorControllerPort = 1;
  public static final int DriverControllerPort = 0;

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

  // Field Related Constants
  public static final double kHighGoalHeight = 96.25; // Center Goal Height
}
