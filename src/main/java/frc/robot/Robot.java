package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import frc.robot.vision.AimingParameters;
import java.util.Optional;
import frc.robot.util.loops.Looper;
import frc.robot.util.Util;


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
    INTAKE_WAITING,
    INTAKE,
    STORE_BALL,
    STORAGE_COMPLETE,
    STORAGE_EJECT
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

  public enum TurretState {
    AUTO_AIM,
    MANUAL
  }

  private final int AUTONOMOUS_BALL_COUNT = 3;
  private final double FIRE_DURATION_SECONDS = 0.3;
  private final int BARF_TIMER_DURATION = 3;

  private State mState = State.IDLE;
  private IntakeState mIntakeState = IntakeState.IDLE;
  private ShootingState mShootingState = ShootingState.IDLE;
  private ClimbingState mClimbingState = ClimbingState.IDLE;
  private TurretState mTurretState = TurretState.AUTO_AIM;

  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  // Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Storage mStorage = Storage.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Turret mTurret = Turret.getInstance();
  private final Drive mDrive = Drive.getInstance();

  //Looper - Running on a set period
  private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);


  private boolean startedHoming = false;
  private BallIndicator mBallIndicator;
  private CameraManager mCameraManager;

  private final RobotTracker mRobotTracker = RobotTracker.getInstance();
  private final RobotTrackerUpdater mRobotTrackerUpdater = RobotTrackerUpdater.getInstance();

  public Relay visionLight = new Relay(0);

  // Control Variables
  private LatchedBoolean AutoAim = new LatchedBoolean();
  private LatchedBoolean HarvestAim = new LatchedBoolean();
  static NetworkTable mTable;

  private boolean mIsSpinningUp = false;

  // Fire timer for shooter
  private Timer mFireTimer = new Timer();
  private Timer mBarfTimer = new Timer();
  Logger mRobotLogger = new Logger("robot");

  // Shooter velocity trim state
  LatchedBoolean mShooterVelocityTrimUp = new LatchedBoolean();
  LatchedBoolean mShooterVelocityTrimDown = new LatchedBoolean();

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    // Zero all nesscary sensors on Robot
    Config.getInstance().reload();
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());

    
    //Register the Enabled Looper
    //Used to run background tasks!
    //Constantly collects information
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);


    // Zero all nesscary sensors on Robot
    mSubsystemManager.zeroSensors();
    visionLight.set(Relay.Value.kForward);

    // Reset Robot Tracker - Note starting position of the Robot
    // This starting Rotation, X, Y is now the Zero Point
    mRobotTracker.reset();


    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");


    mCameraManager = CameraManager.getInstance();
    mCameraManager.init();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_LEDS)) {
      mBallIndicator = BallIndicator.getInstance();
    }
  }

  
  @Override
  public void robotPeriodic() {}

  private void updateSmartDashboard() {
    RobotTracker.RobotTrackerResult result = mRobotTracker.GetTurretError(Timer.getFPGATimestamp());

    SmartDashboard.putBoolean("Manual Spin-up", mIsSpinningUp);
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());
    SmartDashboard.putBoolean("Has Vision", result.HasResult);
    if (result.HasResult) {
      SmartDashboard.putNumber("Turret Offset Error", -result.turret_error.getDegrees());
    } else {
      SmartDashboard.putNumber("Turret Offset Error", 0);
    }

    SmartDashboard.putNumber("Ball Counter", mStorage.getBallCount());
    SmartDashboard.putBoolean("ShooterFull", mStorage.isFull());
    SmartDashboard.putBoolean("ShooterSpunUp", mShooter.isAtVelocity());
    SmartDashboard.putNumber("ElevateTrim", mShooter.getVelocityAdjustment());

    SmartDashboard.putString("RobotState", mState.name());
    SmartDashboard.putString("IntakeState", mIntakeState.name());
    SmartDashboard.putString("ShootingState", mShootingState.name());
    SmartDashboard.putString("ClimbingState", mClimbingState.name());
    
    SmartDashboard.putBoolean("Garage Door", mStorage.getIntakeSensor());
    SmartDashboard.putNumber("Shooter Speed", mShooter.getSpeed());
  }

  @Override
  public void autonomousInit() {
    mIsSpinningUp = false;
    mOperatorInterface.checkControllers();
    
    mRobotLogger.log("Auto Init Called");

    //Start background looper
    //collections information periodically
    mEnabledLooper.start();

    Config.getInstance().reload();

    mState = State.SHOOTING;
    mShootingState = ShootingState.PREPARE_TO_SHOOT;
    mStorage.preloadBalls(AUTONOMOUS_BALL_COUNT);
  }

  @Override
  public void autonomousPeriodic() {
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    visionLight.set(Relay.Value.kForward);
    mIsSpinningUp = false;
    mRobotLogger.log("Teleop Init!");

    //Start background looper
    //collections information periodically
    mEnabledLooper.start();

    Config.getInstance().reload();

    mStorage.preloadBalls(0);

    mOperatorInterface.checkControllers();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;


    // updated in Intake.java
    SmartDashboard.putBoolean("Intake Spinning Up", false);
    SmartDashboard.putBoolean("Intake Overcurrent", false);
    SmartDashboard.putBoolean("Intake Overcurrent Debounced", false);
    SmartDashboard.putNumber("Intake Current", 0);
    SmartDashboard.putNumber("Intake Current Countdown", 0);
    SmartDashboard.putNumber("Encoder Distance", 0);
    SmartDashboard.putNumber("Encoder Distance Raw", 0);
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
    } else if (mOperatorInterface.isBarf()) {
      mIntake.barf();
    } else {
      mIntake.stop();
    }

    if (mOperatorInterface.isBarf()) {
      mStorage.barf();
    } else {
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

    if (mOperatorInterface.isDriveLeftBackTest()) {
      mDrive.setOutputLeftBack(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputLeftBack(0);
    }

    if (mOperatorInterface.isDriveLeftFrontTest()) {
      mDrive.setOutputLeftFront(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputLeftFront(0);
    }

    if (mOperatorInterface.isDriveRightBackTest()) {
      mDrive.setOutputRightBack(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputRightBack(0);
    }

    if (mOperatorInterface.isDriveRightFrontTest()) {
      mDrive.setOutputRightFront(mOperatorInterface.getDriveThrottle());
    } else {
      mDrive.setOutputRightFront(0);
    }

    System.out.println(mStorage.getEncoder());
    if (mOperatorInterface.isClimberTest()) {
      mClimber.jog(mOperatorInterface.getClimberJogSpeed());
    }
    
    if (mOperatorInterface.isHomeClimber()) {
      startedHoming = true;
      mClimber.home();
    } else {
      if (startedHoming) {
        mClimber.stop();
        startedHoming = false;
      }
    }
  }

  @Override
  public void disabledInit() {
    visionLight.set(Relay.Value.kForward);

    Config.getInstance().reload();
  }

  @Override
  public void disabledPeriodic() {

  }

  //turret loop
  //constantly commands the turret with vision or manual controls
  public void turretLoop() {

    if(mTurretState == TurretState.AUTO_AIM){
      //Command the Turret with vision set points
      RobotTracker.RobotTrackerResult result = mRobotTracker.GetTurretError(Timer.getFPGATimestamp());
        if(result.HasResult){
          //We have Target Information
          mTurret.SetAimError(result.turret_error.getDegrees());
        }else{
          //No Results, Don't Rotate
        }
    }else{
      //Command the Turret Manually
        // Operator Controls
        if (mOperatorInterface.getTurretAdjustLeft()) {
          // manual turret aim
        } else if (mOperatorInterface.getTurretAdjustRight()) {
          // manual turret aim
        }
    }
  }

  public void driveTrainLoop() {
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
    if (prevState != mState) {
      mRobotLogger.log("Changed state to " + mState);
    }
    if (prevIntakeState != mIntakeState) {
      mRobotLogger.log("Changed state to " + mIntakeState);
    }
    if (prevClimbState != mClimbingState) {
      mRobotLogger.log("Changed state to " + mClimbingState);
    }
    if (prevShootState != mShootingState) {
      mRobotLogger.log("Changed state to " + mShootingState);
    }

    if (mOperatorInterface.getStateReset()) {
      mState = State.INTAKE;
      mIntakeState = IntakeState.IDLE;
      mClimbingState = ClimbingState.IDLE;
      mShootingState = ShootingState.IDLE;
      if (mState == State.SHOOTING) {
        mShootingState = ShootingState.SHOOTING_COMPLETE;
      }
      if (mState == State.INTAKE) {
        mIntakeState = IntakeState.IDLE;
      }
    }

    if (mOperatorInterface.isBarf()) {
      mIntakeState = IntakeState.STORAGE_EJECT;
      mBarfTimer.reset();
      mBarfTimer.start();
    }

    // TODO: REMOVE THIS IT SHOULDNT BE HERE
    // check if we are shooting
    // TODO: remove this and only allow shooting if you have at least 1 ball
    checkTransitionToShooting();

    turretLoop();

    driveTrainLoop();

    updateSmartDashboard();


    if(mOperatorInterface.getSpinUp()){
      mIsSpinningUp = !mIsSpinningUp;
    }

    if(mIsSpinningUp){
      mShooter.start();
    }else if(mState != State.SHOOTING){
      mShooter.stop();
    }
 

    // Shooter velocity trim
    if (mShooterVelocityTrimDown.update(mOperatorInterface.getShooterVelocityTrimDown())) {
      mShooter.decreaseVelocity();
    } else if (mShooterVelocityTrimUp.update(mOperatorInterface.getShooterVelocityTrimUp())) {
      mShooter.increaseVelocity();
    } else if (mOperatorInterface.getResetVelocityTrim()) {
      mShooter.resetVelocity();
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
      // TODO: Make this not a transitionary state
      case IDLE:
        mRobotLogger.warn("Intake state is idle");
        mIntake.stop();
        mStorage.stop();
        mShooter.stop();
        mIntakeState = IntakeState.READY_TO_INTAKE;
        checkTransitionToClimbing();
        break;
      case READY_TO_INTAKE:
        checkTransitionToClimbing();

        // If the operator issues the intake command, start intake
        if (mOperatorInterface.startIntake()) {
          mIntake.resetOvercurrentCooldown();
          mIntakeState = IntakeState.INTAKE;
        }
        break;
        // we wait until the garage door sensor is clear before moving to real intake
      case INTAKE_WAITING:
        mIntake.start();
        if (!mStorage.isBallDetected()) {
          mIntakeState = IntakeState.INTAKE;
        }
        break;
      case INTAKE:
        // Check transition to shooting before we start intake of a new ball
        if (!checkTransitionToShooting()) {
          mIntake.start();

          // If a ball is detected, store it
          if (mStorage.isBallDetected()) {
            if(mStorage.getBallCount() == mStorage.getCapacity() + 1){
              mIntakeState = IntakeState.STORAGE_COMPLETE;
            }
            mStorage.updateEncoderPosition();
            mIntakeState = IntakeState.STORE_BALL;
          }

          if (mOperatorInterface.startIntake()) {
            mIntakeState = IntakeState.IDLE;
          }
        }
        break;
      case STORE_BALL:
        mStorage.storeBall();
        mIntake.stop();

        // If the sensor indicates the ball is stored, complete ball storage
        if (mStorage.isBallStored()) {
          mIntakeState = IntakeState.STORAGE_COMPLETE;
        }

        break;
      // we just stored a ball
      case STORAGE_COMPLETE:
        mStorage.addBall();
        mStorage.stop();

        // If the storage is not full, intake another ball
        if (!mStorage.isFull()) {
          mIntakeState = IntakeState.INTAKE_WAITING;
        }

        // Check transition to shooting after storage of ball
        checkTransitionToShooting();

        mIntake.resetOvercurrentCooldown();
        break;
      case STORAGE_EJECT:
        mStorage.updateEncoderPosition();
        mIntake.barf(); // Ball Acqusition Reverse Functionality (BARF)
        mStorage.barf();
        mStorage.emptyBalls();

        if (mBarfTimer.get() >= BARF_TIMER_DURATION) {
          mIntakeState = IntakeState.IDLE;
        }
        break;
      default:
        mRobotLogger.error("Invalid Intake State");
        break;
    }
  }

  private boolean checkTransitionToShooting() {
    RobotTracker.RobotTrackerResult result = mRobotTracker.GetTurretError(Timer.getFPGATimestamp());
    // result.HasResult ensures that our vision system sees a target
    if (mOperatorInterface.getShoot() /* && (!mStorage.isEmpty()) && result.HasResult*/) {
      mRobotLogger.log("Changing to shoot because our driver said so...");
      switch (mState) {

        /** Disables intake if transitioning from intake */
        case INTAKE:
          mIntake.stop();
          mStorage.stop();
          mIntakeState = IntakeState.IDLE;
          break;
        default:
          break;
      }
      mState = State.SHOOTING;

      /** Sets the shooting state to preparing if it's not already */
      if (mShootingState == ShootingState.IDLE) {
        mShootingState = ShootingState.PREPARE_TO_SHOOT;
      }
      return true;
    } else {
      // mRobotLogger.info("Could not shoot because " + (!mStorage.isEmpty()) + " " +
      // mOperatorInterface.getShoot());
      return false;
    }
  }

  private boolean checkTransitionToClimbing() {
    //TODO: Remove the check that climber is enabled
    if (mOperatorInterface.startClimb() && Config.getInstance().getBoolean(Key.CLIMBER__ENABLED)) {
      mRobotLogger.log("Changing to climbing");

      /** Disables intake if transitioning from intake */
      switch (mState) {
        case INTAKE:
          mIntake.stop();
          mStorage.stop();
          mIntakeState = IntakeState.IDLE;
          break;
        default:
          break;
      }

      /** Sets the climbing state to extending if it's not already */
      mState = State.CLIMBING;
      if (mClimbingState == ClimbingState.IDLE) {
        mClimbingState = ClimbingState.EXTENDING;
      }
      return true;
    } else {
      return false;
    }
  }

  private void executeShootingStateMachine() {
    switch (mShootingState) {
      case IDLE:
        checkTransitionToClimbing();
        mRobotLogger.warn("Shooting state is idle");
        mShooter.stop();
        break;
      case PREPARE_TO_SHOOT:
        mStorage.stop();

        /* Starts roller */
        mShooter.start();
        mIsSpinningUp = false;

        /* If rollers are spun up, changes to next state */
        if (mShooter.isAtVelocity() /* TODO: && Target Acquired */) {
          mShootingState = ShootingState.SHOOT_BALL;
          mFireTimer.start();
        }

        if(mOperatorInterface.getShoot()){
          mShootingState = ShootingState.SHOOTING_COMPLETE;
          mStorage.stop();
        }
        break;
      case SHOOT_BALL:
        mStorage.ejectBall();

        mShooter.start();

        if(mOperatorInterface.getShoot()){
          mShootingState = ShootingState.SHOOTING_COMPLETE;
          mStorage.stop();
        }

        /* If finished shooting, changes to next state*/
        if (mShooter.isBallFired()) {
          mShootingState = ShootingState.SHOOT_BALL_COMPLETE;
        }
        break;
      case SHOOT_BALL_COMPLETE:
        /* Decrements storage */
        mStorage.removeBall();

        // We are gonna continue shooting until 
        // /* Goes to complete if storage is empty, otherwise fires again */
        // if (mStorage.isEmpty()) {
        //   mShootingState = ShootingState.SHOOTING_COMPLETE;
        // } else {
        //   mShootingState = ShootingState.PREPARE_TO_SHOOT;
        // }

        
        // shooting is a toggle
        mShootingState = ShootingState.PREPARE_TO_SHOOT;
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
        mClimber.stop();
        mRobotLogger.warn("Climbing state is idle");
        break;
      case EXTENDING:
        //TODO: Decide if climb and retract should be the same button
        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.startClimb()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
        mClimber.extend();

        /** Checks the encoder position to see if it's done climbing */
        if (mClimber.isExtended()) {
          mClimbingState = ClimbingState.EXTENDING_COMPLETE;
        }
      case EXTENDING_COMPLETE:
        mClimber.stop();

        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.startClimb()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
      case RETRACTING:
        mClimber.retract();

        /** Checks the encoder position to see if it's done retracting */
        if (mClimber.isRetracted()) {
          mClimbingState = ClimbingState.IDLE;
        }
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }
  }
}
