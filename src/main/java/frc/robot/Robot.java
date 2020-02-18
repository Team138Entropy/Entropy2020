package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
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
    IDLE, // Default state when State is not INTAKE
    READY_TO_INTAKE,
    INTAKE,
    STORE_BALL,
    STORAGE_COMPLETE
  }

  public enum ShootingState {
    IDLE, // Default state when State is not SHOOTING
    PREPARE_TO_SHOOT,
    SHOOT_BALL,
    SHOOT_BALL_COMPLETE,
    SHOOTING_COMPLETE
  }

  public enum ClimbingState {
    IDLE, // Default state when State is not CLIMBING
    EXTENDING,
    EXTENDING_COMPLETE,
    RETRACTING,
    RETRACTING_COMPLETE
  }

  private final int AUTONOMOUS_BALL_COUNT = 3;

  //TODO: Tune these values
  /** Temporary values start here. */

  /** How long it takes the shooter to fire */
  private final double FIRE_DURATION_SECONDS = 0.5d;

  /** Temporary values end here. */

  private State mState = State.IDLE;
  private IntakeState mIntakeState = IntakeState.IDLE;
  private ShootingState mShootingState = ShootingState.IDLE;
  private ClimbingState mClimbingState = ClimbingState.IDLE;

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
  private final Climber mClimber = Climber.getInstance();
  private BallIndicator mBallIndicator;
  private CameraManager mCameraManager;
  private Drive mDrive;

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
    mSubsystemManager.zeroSensors();
    visionLight.set(Relay.Value.kForward);

    // Reset Robot State - Note starting position of the Robot
    // This starting Rotation, X, Y is now the Zero Point
    EventWatcherThread.getInstance().start();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");
    // mCameraManager = CameraManager.getInstance();
    // mCameraManager.init();
    
    // Reset Robot State
    // Wherever the Robot is now is the starting position
    mRobotState.reset();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

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


  private void updateSmartDashboard() {
    SmartDashboard.putNumber("BallCounter", mStorage.getBallCount());
    SmartDashboard.putBoolean("ShooterFull", mStorage.isFull());
    // TODO: decide if this is necessary and hook it up
    SmartDashboard.putBoolean("ShooterLoaded", false);
    SmartDashboard.putBoolean("ShooterSpunUp", mShooter.isAtVelocity());
    SmartDashboard.putBoolean("TargetLocked", mRobotState.getHighGoalLocked());
    // TODO: haha that was a joke this is the real last one
    SmartDashboard.putNumber("ElevateTrim", 0.0f);

    SmartDashboard.putString("RobotState", mState.name());
    SmartDashboard.putString("IntakeState", mIntakeState.name());
    SmartDashboard.putString("ShootingState", mShootingState.name());
    SmartDashboard.putString("ClimbingState", mClimbingState.name());
  }

  @Override
  public void autonomousInit() {
    mRobotLogger.log("Auto Init Called");

    mStorage.init();
    mDrive.init();

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
    visionLight.set(Relay.Value.kForward);
    mRobotLogger.log("Teleop Init!");

    mStorage.preloadBalls(0);

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

    mStorage.init();
    mDrive.init();

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
    mSubsystemManager.checkSubsystems();
  }

  @Override
  public void testPeriodic() {
    // Intake roller ON while button held
    if (mOperatorInterface.isIntakeRollerTest()) {
      mIntake.setOutput(mOperatorInterface.getOperatorThrottle());
    }else if(mOperatorInterface.isBarf()){
      mIntake.barf();
    } else {
      mIntake.stop();
    }

    if(mOperatorInterface.isBarf()){
      mStorage.barf();
    }else{
      // Bottom storage rollers ON while button held
      if (mOperatorInterface.isStorageRollerBottomTest()) {
        mStorage.setBottomOutput(mOperatorInterface.getOperatorThrottle());
      } else {
        mStorage.setBottomOutput(0);
      }

      // Top storage rollers ON while button held
      if (mOperatorInterface.isStorageRollerTopTest()) {
        mStorage.setTopOutput(mOperatorInterface.getOperatorThrottle());
      } else {
        mStorage.setTopOutput(0);
      }
    }

    // Shooter roller ON while button held
    if (mOperatorInterface.isShooterTest()) {
      mShooter.setOutput(mOperatorInterface.getOperatorThrottle());
    } else {
      mShooter.stop();
    }

    if(mOperatorInterface.isDriveLeftBackTest()) {
      mDrive.setOutputLeftBack(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputLeftBack(0);
    }

    if(mOperatorInterface.isDriveLeftFrontTest()) {
      mDrive.setOutputLeftFront(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputLeftFront(0);
    }

    if(mOperatorInterface.isDriveRightBackTest()) {
      mDrive.setOutputRightBack(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputRightBack(0);
    }

    if(mOperatorInterface.isDriveRightFrontTest()) {
      mDrive.setOutputRightFront(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputRightFront(0);
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
      double driveThrottle = mOperatorInterface.getDriveThrottle();
      double driveTurn = mOperatorInterface.getDriveTurn();
      boolean autoDrive = false;
      mDrive.setDrive(driveThrottle, driveTurn, false);

      // Quickturn
      if (autoDrive == false && mOperatorInterface.getQuickturn()) {
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
        mDrive.setDrive(driveThrottle, driveTurn, false);
      }
    }
  }

  /*
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop() {
    updateSmartDashboard();
    
    State prevState = mState;
    IntakeState prevIntakeState = mIntakeState;
    ClimbingState prevClimbState = mClimbingState;
    ShootingState prevShootState = mShootingState;
    executeRobotStateMachine();
    if(prevState != mState){
      mRobotLogger.log("Changed state to " + mState);
    }
    if(prevIntakeState != mIntakeState){
      mRobotLogger.log("Changed state to " + mIntakeState);
    }
    if(prevClimbState != mClimbingState){
      mRobotLogger.log("Changed state to " + mClimbingState);
    }
    if(prevShootState != mShootingState){
      mRobotLogger.log("Changed state to " + mShootingState);
    }

    // check if we are shooting
    // TODO: remove this and only allow shooting if you have at least 1 ball
    checkTransitionToShooting();

    turretLoop();

    driveTrainLoop();

    updateSmartDashboard();

    // Operator Controls
    if (mOperatorInterface.getTurretAdjustLeft()) {
      // manual turret aim
    } else if (mOperatorInterface.getTurretAdjustRight()) {
      // manual turret aim
    }

    // Camera Swap
    if (mOperatorInterface.getCameraSwap()) {
      // Swap Camera!
    }
  }

  private void executeRobotStateMachine() {
    switch (mState) {
      case IDLE:
        SmartDashboard.putString("RobotState", mState.name());
        SmartDashboard.putString("IntakeState", mIntakeState.name());
        SmartDashboard.putString("ShootingState", mShootingState.name());
        SmartDashboard.putString("ClimbingState", mClimbingState.name());
        mIntake.stop();
        mStorage.stop();
        mShooter.stop();
        mClimber.stop();
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
        mIntake.stop();
        mStorage.stop();
        mShooter.stop();
        mIntakeState = IntakeState.READY_TO_INTAKE;
        break;
      case READY_TO_INTAKE:
        // If the operator issues the intake command, start intake
        if (mOperatorInterface.startIntake()) {
          mIntake.resetOvercurrentCooldown();
          mIntakeState = IntakeState.INTAKE;
        }
        break;
      case INTAKE:
        // Check transition to shooting before we start intake of a new ball
        if (!checkTransitionToShooting()) {
          mIntake.start();

          // If a ball is detected, store it
          if (mIntake.isBallDetected()) {
            mIntakeState = IntakeState.STORE_BALL;
          }
        }
        break;
      case STORE_BALL:
        mStorage.storeBall();
        // TODO: may need to delay stopping the intake roller
        mIntake.stop();

        // If the sensor indicates the ball is stored, complete ball storage
        if (mStorage.wasLineBroke()) {
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
        
        mIntake.resetOvercurrentCooldown();
        break;
      default:
        mRobotLogger.error("Invalid Intake State");
        break;
    }
  }

  private boolean checkTransitionToShooting() {
    // TODO: uncomment this isEmpty when we get ball counting working
    if (mOperatorInterface.getShoot()/* && (!mStorage.isEmpty())*/) {
      mRobotLogger.log("Changing to shoot because our driver said so...");
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
      // mRobotLogger.info("Could not shoot because " + (!mStorage.isEmpty()) + " " + mOperatorInterface.getShoot());
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
        mShooter.stop();
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

        mShooter.start();

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
    switch (mClimbingState) {
      case IDLE:
        mRobotLogger.warn("Climbing state is idle");
        break;
      case EXTENDING:
        //TODO: Decide if climb and retract should be the same button
        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.getClimb()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
        mClimber.extend();

        /** Checks if the motor is overcurrenting, signalling it's done climbing */
        if (mClimber.checkOvercurrent()) {
          mClimbingState = ClimbingState.EXTENDING_COMPLETE;
        }
      case EXTENDING_COMPLETE:
        mClimber.stop();

        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.getClimb()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
      case RETRACTING:
        mClimber.retract();

        /** Checks if the motor is overcurrenting, signalling it's done retracting */
        if (mClimber.checkOvercurrent()) {
          mClimbingState = ClimbingState.RETRACTING_COMPLETE;
        }
      case RETRACTING_COMPLETE:
        mClimber.stop();
        mClimbingState = ClimbingState.IDLE;

        //TODO: Decide whether the robot go idle after it's done climbing
        /** Changes the robot state to idle */
        mState = State.IDLE;
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }
  }
}
