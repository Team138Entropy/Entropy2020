package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.auto.IntakeSegment;
import frc.robot.auto.Path;
import frc.robot.auto.Paths;
import frc.robot.auto.ShootSegment;
import frc.robot.auto.SyncIntakeSegment;
import frc.robot.auto.VisionToggleSegment;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import frc.robot.util.loops.Looper;
import frc.robot.vision.VisionPacket;
import java.io.IOException;
import java.net.InetAddress;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // Modes
  public enum Mode {
    Sharpshooter,
    Rebounder,
    Climber
  }

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
    WAIT,
    EXTENDING,
    EXTENDING_COMPLETE,
    HOLD,
    RETRACTING,
    RETRACTING_COMPLETE
  }

  public enum TurretState {
    AUTO_AIM,
    MANUAL
  }

  public enum DriveState {
    MANUAL,
    AUTO_DRIVE
  }

  // Vision Related Valuables
  private double LastDistance = -1;
  private int LastTurretVisionID = -1; // use IDs to filter out bad ideas
  private int LastFeederStationVisionID = -1;

  private double mTurretAdjust = 0;

  public enum TestState {
    START,
    TEST_PI,
    TEST_LIGHT,
    INTAKE_FORWARD,
    INTAKE_BACKWARD,
    STORAGE_ENCODER_FORWARDS_TEST,
    STORAGE_ENCODER_FORWARDS_TEST_WAITING,
    STORAGE_ENCODER_STOP,
    STORAGE_ENCODER_BACKWARDS_TEST,
    STORAGE_ENCODER_BACKWARDS_TEST_WAITING,
    STORAGE_ENCODER_NO_ENCODER_FORWARDS_TEST,
    STORAGE_ENCODER_NO_ENCODER_BACKWARDS_TEST,
    SHOOTER_ENCODER_TEST,
    SHOOTER_ENCODER_TEST_WAITING,
    DRIVE_LEFT_FRONT,
    DRIVE_LEFT_BACK,
    DRIVE_RIGHT_FRONT,
    DRIVE_RIGHT_BACK,
    MANUAL
  }

  private boolean mIsPracticeBot = true;

  private final int AUTONOMOUS_BALL_COUNT = 3;
  private final double FIRE_DURATION_SECONDS = 0.3;
  private final int BARF_TIMER_DURATION = 3;

  private final int ShotCooldown = 15; // each loop is 20 secs.. 200 ms cooldown
  private int mCurrentCooldown = ShotCooldown;

  private State mState = State.IDLE;
  private IntakeState mIntakeState = IntakeState.IDLE;
  private ShootingState mShootingState = ShootingState.IDLE;
  private ClimbingState mClimbingState = ClimbingState.IDLE;
  private TurretState mTurretState = TurretState.MANUAL;
  private DriveState mDriveState = DriveState.MANUAL;
  private TestState mTestState;

  // Auto stuff
  private static boolean mAuto = false;
  private boolean mShooterIsStopped = false;
  private Path mAutoPath = Paths.NO_OP;

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

  private static final DigitalInput practiceInput =
      new DigitalInput(Config.getInstance().getInt(Key.ROBOT__PRACTICE_JUMPER_PIN));

  private static boolean isPracticeBot = false;

  // Looper - Running on a set period
  private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);

  private boolean startedHoming = false;
  private BallIndicator mBallIndicator;
  private CameraManager mCameraManager;

  private final RobotTracker mRobotTracker = RobotTracker.getInstance();
  private final RobotTrackerUpdater mRobotTrackerUpdater = RobotTrackerUpdater.getInstance();

  /**
   * The robot's gyro. Don't use this for absolute measurements. See {@link #getGyro()} for more
   * details.
   *
   * @see #getGyro()
   */
  private static ADXRS450_Gyro sGyro = new ADXRS450_Gyro();

  public Relay visionLight = new Relay(0);

  // Control Variables
  private LatchedBoolean AutoAim = new LatchedBoolean();
  private LatchedBoolean HarvestAim = new LatchedBoolean();
  static NetworkTable mTable;

  private static boolean sIsSpinningUp = false;

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

  private int mStartingStorageEncoderPosition;
  private int mTestPosition;
  private Timer mTestTimer = new Timer();

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Auto Layout", 0);

    // Zero all nesscary sensors on Robot
    Config.getInstance().reload();
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());

    // Read the jumper pin for practice bot
    readIsPracticeBot();

    // Register the Enabled Looper
    // Used to run background tasks!
    // Constantly collects information
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);

    // Zero all nesscary sensors on Robot
    mSubsystemManager.zeroSensors();

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

    sGyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());
  }

  private void updateSmartDashboard() {

    mClimber.updateSmartDashboard();

    SmartDashboard.putBoolean("Intake Mode", mState == State.INTAKE&& mIntakeState != IntakeState.IDLE && mIntakeState != IntakeState.READY_TO_INTAKE);
    SmartDashboard.putBoolean("Shooting Mode", mState == State.SHOOTING);
    SmartDashboard.putBoolean("Climbing Mode", mState == State.CLIMBING);

    SmartDashboard.putBoolean("Practice Bot", getIsPracticeBot());
    SmartDashboard.putString("Turret State", mTurretState.toString());

    SmartDashboard.putBoolean("Manual Spin-up", sIsSpinningUp);
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());
    /*
     * SmartDashboard.putBoolean("Has Vision", result.HasResult); if
     * (result.HasResult) { SmartDashboard.putNumber("Turret Offset Error",
     * -result.turret_error.getDegrees()); } else {
     * SmartDashboard.putNumber("Turret Offset Error", 0); }
     */
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

    SmartDashboard.putNumber("Vision Distance", LastDistance);
    SmartDashboard.putBoolean("Has Vision", LastDistance != -1);

    SmartDashboard.putNumber("Garage As Number", mStorage.getIntakeSensor() ? 1 : 0);
  }

  @Override
  public void autonomousInit() {
    mShooter.stop();
    int autoMode = (int) Math.round(SmartDashboard.getNumber("Auto Layout", 1));

    mAuto = true;
    sIsSpinningUp = false;
    mOperatorInterface.checkControllers();

    mRobotLogger.log("Auto Init Called " + autoMode);

    // Start background looper
    // collections information periodically
    mEnabledLooper.start();

    Config.getInstance().reload();

    mState = State.SHOOTING;
    mShootingState = ShootingState.IDLE;
    mIntakeState = IntakeState.IDLE;
    mStorage.preloadBalls(AUTONOMOUS_BALL_COUNT);

    mAutoPath = Paths.find("comp4").orElse(Paths.NO_OP);
    mShooterIsStopped = false;
    IntakeSegment
        .resetActivatedState(); // In case we didn't cleanly finish for some reason (emergency
    // stop?)
    ShootSegment.resetState();
  }

  @Override
  public void autonomousPeriodic() {
    mAutoPath.tick();

    if (ShootSegment.shouldStartShooting()) {
      mRobotLogger.info("Setting shooting state to prepare to shoot");
      mShootingState = ShootingState.PREPARE_TO_SHOOT;
    }

    if (mStorage.isEmpty() && !mShooterIsStopped) {
      mShooterIsStopped = true;
      mRobotLogger.info("Setting shooting state to complete");
      mShootingState = ShootingState.SHOOTING_COMPLETE;
      mStorage.stop();
    }

    if (IntakeSegment.isActive() || SyncIntakeSegment.isActive()) {
      executeIntakeStateMachine();
    }

    if (VisionToggleSegment.getToggle()) {
      toggleVision();
    }

    if (sIsSpinningUp) {
      mShooter.start();
    } else if (mState != State.SHOOTING) {
      mShooter.stop();
    }

    executeShootingStateMachine();

    updateSmartDashboard();

    turretLoop();
  }

  @Override
  public void teleopInit() {
    mClimber.resetEncoder();

    visionLight.set(Relay.Value.kOff);
    mAuto = false;
    sIsSpinningUp = false;
    mRobotLogger.log("Teleop Init!");

    // Start background looper
    // collections information periodically
    mEnabledLooper.start();

    Config.getInstance().reload();

    mOperatorInterface.checkControllers();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

    mDrive.zeroEncoders();

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

    int left = mDrive.getLeftEncoderDistance();
    int right = -mDrive.getRightEncoderDistance();
    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);

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
    mAuto = false;
    mTestState = TestState.MANUAL;

    Config.getInstance().reload();
    mSubsystemManager.checkSubsystems();

    SmartDashboard.putBoolean("Raspberry PI Passed", false);
    SmartDashboard.putBoolean("Intake Forwards Passed", false);
    SmartDashboard.putBoolean("Intake Backwards Passed", false);
    SmartDashboard.putBoolean("Storage Forwards Test Passed", false);
    SmartDashboard.putBoolean("Storage Backwards Test Passed", false);
    SmartDashboard.putBoolean("Storage No Encoder Forwards Test Passed", false);
    SmartDashboard.putBoolean("Storage No Encoder Backwards Test Passed", false);
    SmartDashboard.putBoolean("Shooter Initial Speed Test Passed", false);
    SmartDashboard.putBoolean("Shooter Speed Test Passed", false);
    SmartDashboard.putBoolean("Drive Left Front Passed", false);
    SmartDashboard.putBoolean("Drive Left Back Passed", false);
    SmartDashboard.putBoolean("Drive Right Front Passed", false);
    SmartDashboard.putBoolean("Drive Right Back Passed", false);
  }

  interface JustAnEncoder {
    default int getEncoder() {
      return 0;
    }
  }

  interface MotorWithEncoder {
    void percentOutput(double output);

    default int getEncoder() {
      return 0;
    }
  }

  private void setupMotorTest(JustAnEncoder func) {
    mTestPosition = func.getEncoder();

    mTestTimer.reset();
    mTestTimer.start();
  }

  private boolean runMotorTest(
      MotorWithEncoder func,
      String name,
      boolean hasEncoder,
      int expectedPosition,
      int acceptableError,
      double testTime) {

    func.percentOutput(1d);

    if (mTestTimer.get() >= testTime) {
      if (hasEncoder) {
        int deltaPosition = func.getEncoder() - mTestPosition;
        int error = Math.abs(deltaPosition - expectedPosition);
        SmartDashboard.putNumber(name + " Delta", deltaPosition);
        SmartDashboard.putNumber(name + " Error", error);
        if (error > acceptableError) {
          SmartDashboard.putBoolean(name + " Passed", false);
        } else {
          SmartDashboard.putBoolean(name + " Passed", true);
        }
      } else {
        SmartDashboard.putBoolean(name + " Passed", true);
      }

      func.percentOutput(0d);

      mTestTimer.reset();
      mTestTimer.start();

      return true;
    }
    return false;
  }

  @Override
  public void testPeriodic() {
    double timePerTest = Config.getInstance().getDouble(Key.TESTMODE__TIME_PER_TEST);
    int expectedStorageDistance =
        Config.getInstance().getInt(Key.TESTMODE__EXPECTED_STORAGE_DISTANCE);
    int storageAcceptableError =
        Config.getInstance().getInt(Key.TESTMODE__STORAGE_ACCEPTABLE_ERROR);
    int expectedShooterSpeed = Config.getInstance().getInt(Key.TESTMODE__EXPECTED_SHOOTER_SPEED);
    int shooterAcceptableError =
        Config.getInstance().getInt(Key.TESTMODE__SHOOTER_ACCEPTABLE_ERROR);
    SmartDashboard.putString("Test State", mTestState.toString());
    SmartDashboard.putBoolean("Driver Cameras", mCameraManager.getCameraStatus());
    SmartDashboard.putBoolean("Garage Door", mStorage.getIntakeSensor());
    SmartDashboard.putNumber("Climber Position", mClimber.getEncoderPosition());
    SmartDashboard.putNumber("Storage encoder", mStorage.getEncoder());

    switch (mTestState) {
      case START:
        mTestTimer.reset();
        mTestTimer.start();
        mTestState = TestState.TEST_PI;
        // mTestState = TestState.DRIVE1_FORWARD_TEST;
        mStorage.updateEncoderPosition();
        break;
      case TEST_PI:
        try {
          InetAddress address = InetAddress.getByName("10.1.38.41");
          boolean reachable = address.isReachable(1000);
          if (reachable) {
            SmartDashboard.putBoolean("Raspberry PI Passed", true);
          }
        } catch (IOException e) {
          e.printStackTrace();
        }
        mTestState = TestState.TEST_LIGHT;
        break;
      case TEST_LIGHT:
        visionLight.set(Relay.Value.kForward);
        mTestState = TestState.INTAKE_FORWARD;
        mTestTimer.reset();
        mTestTimer.start();
        break;
      case INTAKE_FORWARD:
        if (runMotorTest(mIntake::setOutput, "Intake Forwards", false, 0, 0, timePerTest)) {
          mTestState = TestState.INTAKE_BACKWARD;
        }
        break;
      case INTAKE_BACKWARD:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                mIntake.setOutput(-output);
              }
            },
            "Intake Backwards",
            false,
            0,
            0,
            timePerTest)) {
          mTestState = TestState.STORAGE_ENCODER_FORWARDS_TEST;
        }
        break;
      case STORAGE_ENCODER_FORWARDS_TEST:

        // intentional fallthrough
      case STORAGE_ENCODER_BACKWARDS_TEST:
        // mRobotLogger.log("Got initial encoder value " + mStorage.getEncoder());
        // mStartingStorageEncoderPosition = mStorage.getEncoder();

        // mTestTimer.reset();
        // mTestTimer.start();

        if (mTestState == TestState.STORAGE_ENCODER_FORWARDS_TEST) {
          mTestState = TestState.STORAGE_ENCODER_FORWARDS_TEST_WAITING;
        } else {
          mTestState = TestState.STORAGE_ENCODER_BACKWARDS_TEST_WAITING;
        }

        setupMotorTest(
            new JustAnEncoder() {
              @Override
              public int getEncoder() {
                return mStorage.getEncoder();
              }
            });
        break;
      case STORAGE_ENCODER_FORWARDS_TEST_WAITING:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public int getEncoder() {
                return mStorage.getEncoder();
              }

              @Override
              public void percentOutput(double output) {
                if (mIsPracticeBot) {
                  mStorage.setBottomOutput(output);
                } else {
                  mStorage.setTopOutput(output);
                }
              }
            },
            "Storage Forwards Test",
            true,
            expectedStorageDistance,
            storageAcceptableError,
            timePerTest)) {
          mTestState = TestState.STORAGE_ENCODER_STOP;
        }
        break;
      case STORAGE_ENCODER_STOP:
        if (mTestTimer.get() >= timePerTest) {
          mTestTimer.reset();
          mTestTimer.start();
          mTestState = TestState.STORAGE_ENCODER_BACKWARDS_TEST;
        }
        break;
      case STORAGE_ENCODER_BACKWARDS_TEST_WAITING:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public int getEncoder() {
                return mStorage.getEncoder();
              }

              @Override
              public void percentOutput(double output) {
                if (mIsPracticeBot) {
                  mStorage.setBottomOutput(-output);
                } else {
                  mStorage.setTopOutput(-output);
                }
              }
            },
            "Storage Backwards Test",
            true,
            -expectedStorageDistance,
            storageAcceptableError,
            timePerTest)) {
          mTestState = TestState.STORAGE_ENCODER_NO_ENCODER_FORWARDS_TEST;
        }
        break;
      case STORAGE_ENCODER_NO_ENCODER_FORWARDS_TEST:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                if (mIsPracticeBot) {
                  mStorage.setTopOutput(output);
                } else {
                  mStorage.setBottomOutput(output);
                }
              }
            },
            "Storage No Encoder Forwards Test",
            false,
            0,
            0,
            timePerTest)) {
          mTestState = TestState.STORAGE_ENCODER_NO_ENCODER_BACKWARDS_TEST;
        }
        break;
      case STORAGE_ENCODER_NO_ENCODER_BACKWARDS_TEST:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                if (mIsPracticeBot) {
                  mStorage.setTopOutput(-output);
                } else {
                  mStorage.setBottomOutput(-output);
                }
              }
            },
            "Storage No Encoder Backwards Test",
            false,
            0,
            0,
            timePerTest)) {
          mTestState = TestState.SHOOTER_ENCODER_TEST;
        }
        break;
      case SHOOTER_ENCODER_TEST:
        setupMotorTest(new JustAnEncoder() {});

        if (mShooter.getSpeed() != 0) {
          SmartDashboard.putBoolean("Shooter Initial Speed Test Passed", false);
        } else {
          SmartDashboard.putBoolean("Shooter Initial Speed Test Passed", true);
        }

        mTestState = TestState.SHOOTER_ENCODER_TEST_WAITING;
        break;
      case SHOOTER_ENCODER_TEST_WAITING:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public int getEncoder() {
                return mShooter.getSpeed();
              }

              @Override
              public void percentOutput(double output) {
                mShooter.setOutput(output);
              }
            },
            "Shooter Speed Test",
            true,
            expectedShooterSpeed,
            shooterAcceptableError,
            timePerTest)) {
          mTestState = TestState.DRIVE_LEFT_FRONT;

          setupMotorTest(
              new JustAnEncoder() {
                @Override
                public int getEncoder() {
                  return (int) mDrive.getLeftEncoderDistance();
                }
              });
        }
        break;
      case DRIVE_LEFT_FRONT:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                mDrive.setOutputLeftFront(output / 2);
              }

              @Override
              public int getEncoder() {
                return (int) mDrive.getLeftEncoderDistance();
              }
            },
            "Drive Left Front",
            true,
            1139,
            200,
            timePerTest)) {
          mTestState = TestState.DRIVE_LEFT_BACK;
          setupMotorTest(
              new JustAnEncoder() {
                @Override
                public int getEncoder() {
                  return (int) mDrive.getLeftEncoderDistance();
                }
              });
        }
        break;
      case DRIVE_LEFT_BACK:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                mDrive.setOutputLeftBack(output / 2);
              }

              @Override
              public int getEncoder() {
                return (int) mDrive.getLeftEncoderDistance();
              }
            },
            "Drive Left Back",
            true,
            1231,
            200,
            timePerTest)) {
          mTestState = TestState.DRIVE_RIGHT_FRONT;
          setupMotorTest(
              new JustAnEncoder() {
                @Override
                public int getEncoder() {
                  return (int) mDrive.getRightEncoderDistance();
                }
              });
        }
        break;
      case DRIVE_RIGHT_FRONT:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                mDrive.setOutputRightFront(output / 2);
              }

              @Override
              public int getEncoder() {
                return (int) mDrive.getRightEncoderDistance();
              }
            },
            "Drive Right Front",
            true,
            873,
            200,
            timePerTest)) {
          mTestState = TestState.DRIVE_RIGHT_BACK;
          setupMotorTest(
              new JustAnEncoder() {
                @Override
                public int getEncoder() {
                  return (int) mDrive.getRightEncoderDistance();
                }
              });
        }
        break;
      case DRIVE_RIGHT_BACK:
        if (runMotorTest(
            new MotorWithEncoder() {
              @Override
              public void percentOutput(double output) {
                mDrive.setOutputRightBack(output / 2);
              }

              @Override
              public int getEncoder() {
                return (int) mDrive.getRightEncoderDistance();
              }
            },
            "Drive Right Back",
            true,
            1485,
            200,
            timePerTest)) {
          mTestState = TestState.MANUAL;
        }
        break;
      case MANUAL:
        if (mOperatorInterface.isClimberTest()) {
          mClimber.jog(
              mOperatorInterface.getClimberJogSpeed()
                  * Config.getInstance().getDouble(Key.CLIMBER__JOG_SPEED_FACTOR));
        } else {
          mClimber.stop();
        }
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

        if (mOperatorInterface.getFunctional()) {
          mTestState = TestState.START;
        }
        break;
      default:
        mRobotLogger.error("Unknown test state " + mTestState.toString());
        break;
    }
    System.out.println("climber position " + mClimber.getEncoderPosition());
  }

  @Override
  public void disabledInit() {
    visionLight.set(Relay.Value.kOff);

    // zero the encoder
    mTurret.zeroSensors();

    // zero turret sensor
    // this assumes the turret is aligned


    Config.getInstance().reload();

    mOperatorInterface.resetOverride();
    mClimber.resetEncoder();
  }

  @Override
  public void disabledPeriodic() {}

  // turret loop
  // constantly commands the turret with vision or manual controls
  public synchronized void turretLoop() {
    SmartDashboard.putNumber("Vision Turret Adjust", mTurretAdjust);
    if (mTurretState == TurretState.AUTO_AIM) {
      double turretAdjust = mOperatorInterface.getTurretAdjust();
      mTurretAdjust += turretAdjust * Constants.TURRET_MANUAL_ADJUST_FACTOR;

      // Command the Turret with vision set points
      // RobotTracker.RobotTrackerResult result =
      // mRobotTracker.GetTurretError(Timer.getFPGATimestamp());
      // double dis = result.distance;
      VisionPacket vp = mRobotTracker.GetTurretVisionPacket(Timer.getFPGATimestamp());
      if (vp.HasValue == true) {
        // We have Target Information
        LastDistance = vp.Distance;

        // verify we haven't already commanded this packet!
        if (vp.ID != LastTurretVisionID) {
          mTurret.SetAimError(vp.Error_Angle + (vp.getTurretOffset()* -1) + mTurretAdjust);
          LastTurretVisionID = vp.ID;
        }

      } else {
        // No Results, Don't Rotate

      }
    } else {
      // Command the Turret Manually
      // Operator Controls
      double ManualTurn = mOperatorInterface.getTurretAdjust();
      if (Math.abs(ManualTurn) > .2) {
        mTurret.SetManualOutput(mOperatorInterface.getTurretAdjust());
      } else {
        mTurret.SetManualOutput(0);
      }
    }
  }

  public void driveTrainLoop() {
    // TODO: Cache whether or not the robot has a drivetrain. We shouldn't be calling the config
    // system every tick.
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)) {
      // Check User Inputs
      double driveThrottle = mOperatorInterface.getDriveThrottle();
      double driveTurn = mOperatorInterface.getDriveTurn();

      boolean WantsAutoAim = mOperatorInterface.getFeederSteer();

      // Continue Driving
      if (WantsAutoAim == true) {
        // Harvest Mode - AutoSteer Functionality
        // Used for tracking a ball
        // we may want to limit the speed?
        // RobotTracker.RobotTrackerResult DriveResult =
        // mRobotTracker.GetFeederStationError(Timer.getFPGATimestamp());
        mDriveState = DriveState.AUTO_DRIVE;

        VisionPacket vp = mRobotTracker.GetTurretVisionPacket(Timer.getFPGATimestamp());
        // mDrive.autoSteerFeederStation(driveThrottle, vp.Error_Angle);
      } else {
        // Standard Manual Drive
        mDrive.setDrive(driveThrottle, driveTurn, false);

        // if we were previously in auto drive.. turn it off
        if (mDriveState == DriveState.AUTO_DRIVE) {
          mDriveState = DriveState.MANUAL;
        }
      }
    }
  }

  public void toggleVision() {
    if (mTurretState == TurretState.AUTO_AIM) {
      // Turn off Auto Aiming
      visionLight.set(Relay.Value.kOff);
      mTurretState = TurretState.MANUAL;
    } else if (mTurretState == TurretState.MANUAL) {
      // Turn on Auto Aiming
      mTurretState = TurretState.AUTO_AIM;

      // Enable Light
      visionLight.set(Relay.Value.kForward);
    }
  }

  /*
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop() {
    if (mOperatorInterface.getVisionToggle()) {
      toggleVision();
    }

    mShooter.updateDistance(LastDistance);

    turretLoop();

    driveTrainLoop();

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
      if (mIntakeState == IntakeState.STORAGE_EJECT) {
        mIntakeState = IntakeState.IDLE;
      } else {
        mIntakeState = IntakeState.STORAGE_EJECT;
        mBarfTimer.reset();
        mBarfTimer.start();
      }
    }

    // TODO: REMOVE THIS IT SHOULDNT BE HERE
    // check if we are shooting
    // TODO: remove this and only allow shooting if you have at least 1 ball
    checkTransitionToShooting();

    updateSmartDashboard();

    if (mOperatorInterface.getSpinUp()) {
      sIsSpinningUp = !sIsSpinningUp;
    }

    // spin up shooter if commanded
    if (sIsSpinningUp) {
      mShooter.start();
    } else if (mState != State.SHOOTING) {
      mShooter.stop();
    }

    // Shooter velocity trim
    if (mShooterVelocityTrimDown.update(mOperatorInterface.getShooterVelocityTrimDown())) {
      // mShooter.decreaseVelocity();
    } else if (mShooterVelocityTrimUp.update(mOperatorInterface.getShooterVelocityTrimUp())) {
      // mShooter.increaseVelocity();
    } else if (mOperatorInterface.getResetVelocityTrim()) {
      mShooter.resetVelocity();
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
        checkTransitionToClimbing();
        break;
      case INTAKE:
        executeIntakeStateMachine();
        checkTransitionToClimbing();
        break;
      case SHOOTING:
        executeShootingStateMachine();
        checkTransitionToClimbing();
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
            if (mStorage.getBallCount() == mStorage.getCapacity() + 1) {
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
    // TODO: Remove the check that climber is enabled
    if (mOperatorInterface.climbStart() && Config.getInstance().getBoolean(Key.CLIMBER__ENABLED)) {
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

      /** Sets the climbing state to idle if it's not already */
      mState = State.CLIMBING;

      mDrive.setClimbingSpeed(true);

      if (mClimbingState == ClimbingState.IDLE) {
        mClimbingState = ClimbingState.WAIT;
      }
      return true;
    } else {
      return false;
    }
  }

  private boolean checkEscapeClimbHold() {
    if (mOperatorInterface.climbStart()) {
      mState = State.INTAKE;

      mIntakeState = IntakeState.READY_TO_INTAKE;

      mDrive.setClimbingSpeed(false);

      return true;
    } else {
      return false;
    }
  }

  private void executeShootingStateMachine() {
    switch (mShootingState) {
      case IDLE:
        mRobotLogger.warn("Shooting state is idle");
        mShooter.stop();
        break;
      case PREPARE_TO_SHOOT:
        mStorage.stop();

        /* Starts roller */
        mShooter.start();
        sIsSpinningUp = false;

        // make robot pass though cooldown timer
        // should help between shots and solve low velocity issue
        if (mCurrentCooldown > 0) {
          mCurrentCooldown--;
          return; // skip the rest of this loop
        }

        /* If rollers are spun up, changes to next state */
        if (mShooter.isAtVelocity() /* TODO: && Target Acquired */) {
          mShootingState = ShootingState.SHOOT_BALL;
          // mFireTimer.start();

          // reset cooldown timer
          mCurrentCooldown = ShotCooldown;
        }

        if (mOperatorInterface.getShoot()) {
          mShootingState = ShootingState.SHOOTING_COMPLETE;
          mStorage.stop();
        }
        break;
      case SHOOT_BALL:
        mStorage.ejectBall();

        mShooter.start();

        // turn off shooting
        if (mOperatorInterface.getShoot() || mStorage.isEmpty()) {
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
    if(mClimbingState != ClimbingState.IDLE){
      SmartDashboard.putNumber("Climber pos", mClimber.getEncoderPosition());
    }

    switch (mClimbingState) {
      case IDLE:
        mClimber.stop();
        // mRobotLogger.warn("Climbing state is idle");
        break;
      case WAIT:
        if (mOperatorInterface.climbUp()) {
          mClimbingState = ClimbingState.EXTENDING;
        }

        checkEscapeClimbHold();
        break;
      case EXTENDING:
        // TODO: Decide if climb and retract should be the same button
        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.climbUp()) {
          mClimber.extend();
        }else{
          mClimbingState = ClimbingState.HOLD;
        }

        /** Checks the encoder position to see if it's done climbing */
        if (mClimber.isExtended()) {
          mClimbingState = ClimbingState.EXTENDING_COMPLETE;
        }
        break;
      case EXTENDING_COMPLETE:
        mClimber.stop();

        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.climbDown()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
        break;
      case HOLD:
        mClimber.stop();

        if (mOperatorInterface.climbUp()) {
          mClimbingState = ClimbingState.EXTENDING;
        }

        if (mOperatorInterface.climbDown()) {
          mClimbingState = ClimbingState.RETRACTING;
        }

        checkEscapeClimbHold();

        break;
      case RETRACTING:
        mClimber.retract();

        if (mOperatorInterface.climbDown()) {
          mClimber.retract();
        }else{
          mClimbingState = ClimbingState.HOLD;
        }

        /** Checks the encoder position to see if it's done retracting */
        if (mClimber.isRetracted()) {
          mClimbingState = ClimbingState.WAIT;
        }
        break;
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }
    System.out.println(mClimber.getEncoderPosition());
  }

  private static void readIsPracticeBot() {
    // why do we invert it? because
    isPracticeBot = !practiceInput.get();
  }

  public static boolean getIsPracticeBot() {
    return isPracticeBot;
  }

  /**
   * Returns the robot's gyro. It should be noted that this gyro object's reported heading will
   * often <bold>not</bold> reflect the actual heading of the robot. This is because it is reset at
   * the beginning of every autonomous turn segment in order to allow relative turning.
   *
   * @return the gyro
   */
  public static Gyro getGyro() {
    return sGyro;
  }

  public static boolean isAuto() {
    return mAuto;
  }

  public static boolean getSpinningUp() {
    return sIsSpinningUp;
  }

  public static void setSpinningUp(boolean value) {
    sIsSpinningUp = value;
  }
}
