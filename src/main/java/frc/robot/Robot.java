/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // State variables
  public enum State {
    IDLE, // Default state
    INTAKE,
    SHOOTING,
    CLIMBING
  }

  public enum IntakeState {
    IDLE, // Default state, when State is not INTAKE
    READY_TO_INTAKE,
    INTAKE,
    STORE_BALL,
    STORAGE_COMPLETE
  }

  public enum ShootingState {
    IDLE, // Default state, when State is not SHOOTING
    PREPARE_TO_SHOOT,
    SHOOT_BALL,
    SHOOT_BALL_COMPLETE,
    SHOOTING_COMPLETE
  }

  public enum ClimingState {
    IDLE
  }

  private final int AUTONOMOUS_BALL_COUNT = 3;

  private State mState = State.IDLE;
  private IntakeState mIntakeState = IntakeState.IDLE;
  private ShootingState mShootingState = ShootingState.IDLE;
  private ClimingState mClimingState = ClimingState.IDLE;

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

  // Variables from State

  private Turret mTurret;
  static NetworkTable mTable;

  Logger mRobotLogger = new Logger("robot");

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    Config.getInstance().reload();

    mRobotLogger.log("robot init _ 1");

    // Zero all nesscary sensors on Robot
    ZeroSensors();

    EventWatcherThread.getInstance().start();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");
    // Reset Robot State
    // Wherever the Robot is now is the starting position
    mRobotState.reset();

    // Set the initial Robot State
    mState = State.INTAKE;

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

    mState = State.SHOOTING;
    mShootingState = ShootingState.PREPARE_TO_SHOOT;
    mStorage.preloadBalls(AUTONOMOUS_BALL_COUNT);
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

    mState = State.INTAKE;
    mIntakeState = IntakeState.READY_TO_INTAKE;
  }

  @Override
  public void teleopPeriodic() {
    try {
      RobotLoop();
    } catch (Exception e) {
      mRobotLogger.log("RobotLoop Exception");

      // print the exception to the system error
      e.printStackTrace(System.err);
    }
  }

  @Override
  public void testInit() {
    mRobotLogger.log("Entropy 138: Test Init");

    Config.getInstance().reload();
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
      boolean AutoDrive = false;
      mDrive.setDrive(DriveThrottle, DriveTurn, false);

      // Quickturn
      if (AutoDrive == false && mOperatorInterface.getQuickturn()) {
        // Quickturn!
      }
    }
  }

  /*
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop() {
    updateSmartDashboard();

    executeRobotStateMachine();

    turretLoop();

    driveTrainLoop();

    mShooter.periodic();

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

  private void executeRobotStateMachine() {
    switch(mState) {
      case IDLE:
        break;
      case INTAKE:
        executeIntakeStateMachine();
        break;
      case SHOOTING:
        executeShootingStateMachine();
        break;
      case CLIMBING:
        executeClimbingStateMachine();
        break;
      default:
        mRobotLogger.error("Invalid Robot State");
        break;
    }
  }

  private void executeIntakeStateMachine() {
    switch(mIntakeState) {
      case IDLE:
        break;
      case READY_TO_INTAKE:
        if (mOperatorInterface.getLoadChamber()) {
          mIntakeState = IntakeState.INTAKE;
        }
        break;
      case INTAKE:
        // Check transition to shooting before we start intake of a new ball
        if (!checkTransitionToShooting()) {
          mIntake.start();
          if (mStorage.isBallDetected()) {
            mIntakeState = IntakeState.STORE_BALL;
          }
        }
        break;
      case STORE_BALL:
        mStorage.storeBall();
        // TODO: may need to delay stopping the intake roller
        mIntake.stop();
        
        if (mStorage.isBallStored()) {
          mIntakeState = IntakeState.STORAGE_COMPLETE;
        }
        break;
      case STORAGE_COMPLETE:
        // TODO: may need to delay stopping the storage roller
        mStorage.stop();
        mStorage.addBall();

        // Check transition to shooting after storage of ball 
        checkTransitionToShooting();
        break;
      default:
        mRobotLogger.error("Invalid Intake State");
        break;
    }
  }

  private boolean checkTransitionToShooting() {
    if (mOperatorInterface.getShoot() && (!mStorage.isEmpty())) {
      switch (mState) {
        case INTAKE:
          mIntake.stop();
          mStorage.stop();
          mIntakeState = IntakeState.IDLE;
          break;
        default:
          break;
      }
      mState = State.SHOOTING;
      return true;
    }
    else {
      return false;
    }
  }

  private void executeShootingStateMachine() {
    switch(mShootingState) {
      case IDLE:
        break;
      case PREPARE_TO_SHOOT:
        break;
      case SHOOT_BALL:
        break;
      case SHOOT_BALL_COMPLETE:
        break;
      case SHOOTING_COMPLETE:
        break;
      default:
        mRobotLogger.error("Invalid Shooting State");
        break;
    }
  }

  private void executeClimbingStateMachine() {
    switch(mClimingState) {
      case IDLE:
        break;
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }

  }
}
