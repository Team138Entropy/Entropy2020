/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import frc.robot.util.geometry.*;
import frc.robot.vision.AimingParameters;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private Drive mDrive;

  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  // Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Storage mStorage = Storage.getInstance();
  private BallIndicator mBallIndicator;

  private final Compressor mCompressor = new Compressor();

  public Relay visionLight = new Relay(0);

  // Control Variables
  private LatchedBoolean AutoAim = new LatchedBoolean();
  private LatchedBoolean HarvestAim = new LatchedBoolean();
  private Turret mTurret;
  static NetworkTable mTable;

  Logger mRobotLogger = new Logger("robot");

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    // Zero all nesscary sensors on Robot
    Config.getInstance().reload();

    mRobotLogger.log("robot init _ 1");

    // Zero all nesscary sensors on Robot
    ZeroSensors();
    visionLight.set(Relay.Value.kForward);

    // Reset Robot State - Note starting position of the Robot
    // This starting Rotation, X, Y is now the Zero Point
    EventWatcherThread.getInstance().start();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");
    // Reset Robot State
    // Wherever the Robot is now is the starting position
    mRobotState.reset();

    // TODO: remove HAS_TURRET and HAS_DRIVETRAIN
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
      mTurret = Turret.getInstance();
    }

    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)) {
      mDrive = Drive.getInstance();
    }

    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_LEDS)) {
      mBallIndicator = BallIndicator.getInstance();
    }
  }

  /*
    Returns true if the pressure switch reads "low", an undefined value we have no control over.
    Cool!
  */
  public boolean getLowPSI() {
    return mCompressor.getPressureSwitchValue();
  }

  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors() {
    mRobotLogger.log("Zeroing sensors...");
    mSubsystemManager.ZeroSensors();
    mRobotLogger.log("Zeroed sensors");
  }

  private void updateSmartDashboard() {
    // TODO: set this up for real
    SmartDashboard.putString("BallCounter", "BallValue" + " / 5");
    // TODO: change this to the real boolean
    SmartDashboard.putBoolean("ShooterFull", false);
    // TODO: decide if this is necessary and hook it up
    SmartDashboard.putBoolean("ShooterLoaded", false);
    SmartDashboard.putBoolean(
        "ShooterSpunUp", mShooter.getState().equals(Shooter.State.FULL_SPEED));
    // TODO: also, finally, hook this up.
    SmartDashboard.putBoolean("TargetLocked", false);
    // TODO: haha that was a joke this is the real last one
    SmartDashboard.putNumber("ElevateTrim", 0.0f);

    // TODO: cameras will go here eventually
  }

  @Override
  public void autonomousInit() {
    mRobotLogger.log("Auto Init Called");

    Config.getInstance().reload();
  }

  @Override
  public void autonomousPeriodic() {
    mRobotLogger.log("Auto Periodic");
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    mRobotLogger.log("Teleop Init!");

    Config.getInstance().reload();
  }

  @Override
  public void teleopPeriodic() {
    try {
      RobotLoop();
    } catch (Exception e) {
      mRobotLogger.log("RobotLoop Exception: " + e.getMessage());

      // print the exception to the system error
      e.printStackTrace(System.err);
    }
  }

  @Override
  public void testInit() {
    mRobotLogger.log("Entropy 138: Test Init");

    Config.getInstance().reload();
    mSubsystemManager.CheckSubsystems();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
      mTurret.disable();
    }
    Config.getInstance().reload();
  }

  @Override
  public void disabledPeriodic() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
      mRobotLogger.verbose("got pot value of " + mTurret.getPotValue());
    }
  }

  public void turretLoop() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
      mTurret.loop();
    }
  }

  public void driveTrainLoop() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)) {
      // Check User Inputs
      double DriveThrottle = mOperatorInterface.getDriveThrottle();
      double DriveTurn = mOperatorInterface.getDriveTurn();
      boolean DriveShift = mOperatorInterface.getDriveShift();
      boolean AutoDrive = false;
      mDrive.setDrive(DriveThrottle, DriveTurn, false);

      // Quickturn
      if (AutoDrive == false && mOperatorInterface.getQuickturn()) {
        // Quickturn!
      }

      // Detect Harvest Mode
      boolean WantsHarvestMode = mOperatorInterface.getHarvestMode();
      boolean HarvesModePressed = HarvestAim.update(WantsHarvestMode);

      boolean WantsAutoAim = false;

      // Optional Object that may or may not contain a null value
      Optional<AimingParameters> BallAimingParameters; // info to aim to the ball
      Optional<AimingParameters> TargetAimingParameters; // info to aim to the target

      // Continue Driving
      if (WantsHarvestMode == true) {
        // Harvest Mode - AutoSteer Functionality
        // Used for tracking a ball
        // we may want to limit the speed?
        // mDrive.autoSteerBall(DriveThrottle, BallAimingParameters.get());
      } else {
        // Standard Manual Drive
        mDrive.setDrive(DriveThrottle, DriveTurn, false);
      }
    }
  }

  /*
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop() {
    updateSmartDashboard();

    turretLoop();

    driveTrainLoop();

    mShooter.periodic();

    mStorage.periodic();

    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_LEDS)) {
      mBallIndicator.checkTimer();
    }

    // Climb
    if (mOperatorInterface.getClimb()) {
      // climb!
    }

    if (mOperatorInterface.getShoot()) {
      // Shoot
    }
  // Operator Controls
    if (mOperatorInterface.getTurretManual() != -1) {
      // manual turret aim
    }

    // Camera Swap
    if (mOperatorInterface.getCameraSwap()) {
      // Swap Camera!
    }

    // Shoot
    if (mOperatorInterface.getShoot()) {
      // Shoot!
    }

    // Load chamber
    // NOTE: This may or may not be necessary depending on how our sensor pack turns out
    if (mOperatorInterface.getLoadChamber()) {
      // Load chamber!
    }
  }
}
