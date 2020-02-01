package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
  private final double FIRE_DURATION_SECONDS = 0.5;

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
  private CameraManager mCameraManager;

  private Compressor mCompressor;

  public Relay visionLight = new Relay(0);

  // Control Variables
  private LatchedBoolean AutoAim = new LatchedBoolean();
  private LatchedBoolean HarvestAim = new LatchedBoolean();
  private Turret mTurret;
  static NetworkTable mTable;

  // Fire timer for shooter
  private Timer mFireTimer = new Timer();

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
    mCameraManager = CameraManager.getInstance();
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
    Returns true if the pressure switch reads "low", an undefined value we have no control over.
    Cool!
  */
  public boolean getLowPSI() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_COMPRESSOR)) {
      return mCompressor.getPressureSwitchValue();
    } else {
      return false;
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
    SmartDashboard.putString("BallCounter", "BallValue" + mStorage.getBallCount());
    // TODO: change this to the real boolean
    SmartDashboard.putBoolean("ShooterFull", false);
    // TODO: decide if this is necessary and hook it up
    SmartDashboard.putBoolean("ShooterLoaded", false);
    SmartDashboard.putBoolean("ShooterSpunUp", mShooter.isAtVelocity());
    // TODO: also, finally, hook this up.
    SmartDashboard.putBoolean("TargetLocked", false);
    // TODO: haha that was a joke this is the real last one
    SmartDashboard.putNumber("ElevateTrim", 0.0f);

    SmartDashboard.putBoolean("StorageSensor", mStorage.isBallDetected());

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
  public void testPeriodic() {
    // Intake roller ON while button held
    if (mOperatorInterface.isIntakeRollertest()) {
      mIntake.start();
    }
    else {
      mIntake.stop();
    }

    // Storage rollers ON while button held
    if (mOperatorInterface.isStorageRollerTest()) {
      mStorage.storeBall();
    }
    else {
      mStorage.stop();
    }

    // Shooter roller ON while button held
    if (mOperatorInterface.isShooterTest()) {
      mShooter.start();
    }
    else {
      mShooter.stop();
    }
  } 

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

      if (DriveShift) mDrive.SwitchGears();

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

    executeRobotStateMachine();

    turretLoop();

    driveTrainLoop();

    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_LEDS)) {
      mBallIndicator.checkTimer();
    }

    // Climb
    if (mOperatorInterface.getClimb()) {
      // climb!
    }

    // Operator Controls
    if (mOperatorInterface.getTurretAdjustLeft()) {
      // manual turret aim
    }
    else if (mOperatorInterface.getTurretAdjustRight()) {
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
    switch (mState) {
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
    switch (mIntakeState) {
      case IDLE:
        mRobotLogger.warn("Intake state is idle");
        break;
      case READY_TO_INTAKE:
        // If the operator issues the intake command, start intake
        if (mOperatorInterface.getLoadChamber()) {
          mIntakeState = IntakeState.INTAKE;
        }
        break;
      case INTAKE:
        // Check transition to shooting before we start intake of a new ball
        if (!checkTransitionToShooting()) {
          mIntake.start();

          // If a ball is detected, store it
          if (mStorage.isBallDetected()) {
            mIntakeState = IntakeState.STORE_BALL;
          }
        }
        break;
      case STORE_BALL:
        mStorage.storeBall();
        // TODO: may need to delay stopping the intake roller
        mIntake.stop();

        // If the sensor indicates the ball is stored, complete ball storage
        if (mStorage.isBallStored()) {
          mIntakeState = IntakeState.STORAGE_COMPLETE;
        }
        break;
      case STORAGE_COMPLETE:
        mStorage.addBall();

        // TODO: may need to delay stopping the storage roller
        mStorage.stop();

        // If the storage is not full, intake another ball
        if (!mStorage.isFull()) {
          mIntakeState = IntakeState.INTAKE;
        }

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
      if (mShootingState == ShootingState.IDLE) {
        mShootingState = ShootingState.PREPARE_TO_SHOOT;
      }
      return true;
    } else {
      return false;
    }
  }

  /** Returns whether the firing timer has run longer than the duration. */
  public boolean isBallFired() {
    if (mFireTimer.get() >= FIRE_DURATION_SECONDS * 1000) {
      mShooter.stop();
      mFireTimer.stop();
      mFireTimer.reset();
      return true;
    }
    return false;
  }

  private void executeShootingStateMachine() {
    switch (mShootingState) {
      case IDLE:
        mRobotLogger.warn("Shooting state is idle");
        break;
      case PREPARE_TO_SHOOT:

        /* Starts roller */
        mShooter.start();

        // TODO: Placeholder method, replace later.
        mShooter.target();

        /* If rollers are spun up, changes to next state */
        if (mShooter.isAtVelocity() /* TODO: && Target Acquired */) {
          mShootingState = ShootingState.SHOOT_BALL;
        }
        break;
      case SHOOT_BALL:
        mStorage.ejectBall();

        /* If finished shooting, changes to next state*/
        if (isBallFired()) {
          mShootingState = ShootingState.SHOOT_BALL_COMPLETE;
        }
        break;
      case SHOOT_BALL_COMPLETE:
        /* Decrements storage */
        mStorage.removeBall();
        mStorage.stop();

        /* Goes to complete if storage is empty, otherwise fires again */
        if (mStorage.isEmpty()) {
          mShootingState = ShootingState.SHOOTING_COMPLETE;
        } else {
          mShootingState = ShootingState.PREPARE_TO_SHOOT;
        }
        break;
      case SHOOTING_COMPLETE:

        /* Stops the roller and returns to intake state */
        mShooter.stop();
        mShootingState = ShootingState.IDLE;
        mState = State.INTAKE;
        break;
      default:
        mRobotLogger.error("Invalid shooting state");
        break;
    }
  }

  private void executeClimbingStateMachine() {
    switch (mClimingState) {
      case IDLE:
        break;
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }
  }
}
