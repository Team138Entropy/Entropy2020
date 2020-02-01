package frc.robot;

import java.lang.Math;

// Any Sort of Constant or 'Magic Number' should be defined here
public class Constants {
    // Controller Ports
    public static final int OperatorControllerPort = 1;
    public static final int DriverControllerPort = 0;

    //Intake
    public static final int kIntakeTalonPort = 0;    
    public static final double kIntakeRollerPercentOutput = 0.5f;

    //Storage
    public static final int kStorageTalonPort = 0;
    public static final double kStorageRollerPercentOutput = 0.5;

    // Talon Variables
    //Drive
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveId = 2;
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveSlaveId = 4;

    // Solenoid Constants
    public static final int kPCMId = 1;
    public static final int kShifterSolenoidId = 7;

    //Drive Encoder Port
    public static final int kDriveGyroPort = 0;

    // Drive Constants
    public static final double kJoystickThreshold = 0.2;
    public static final double kDriveWheelTrackWidthInches = 22.75;
    public static final double kDriveWheelDiameterInches = 3.938;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters =
            kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0469745223;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps

    //Path Following Constants 
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2

    
    //Vision Tracking Constants
    public static final double kMaxTrackerDistance = 60.0;
    public static final double kMaxGoalTrackAge = 3.0;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
    public static final double kCameraFrameRate = 90.0; //fps



    //Camera Specific Information
    public static final double kCameraDiagonalView = Math.toRadians(75);
    public static final double kCameraHorizontalAspect = 4;
    public static final double kCameraVerticalAspect = 3;
    public static final double kDiagonalAspect = Math.hypot(kCameraHorizontalAspect, kCameraVerticalAspect);
    public static final double kCameraHorizontalView = Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraHorizontalAspect / kCameraDiagonalView)) * 2;
    public static final double kCameraVerticalView = Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraVerticalAspect / kCameraDiagonalView)) * 2;


    //Field Related Constants
    public static final double kHighGoalHeight = 96.25; //Center Goal Height
}
