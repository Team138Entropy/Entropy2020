package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Logger;
import frc.robot.util.*;
import frc.robot.util.geometry.*;
import frc.robot.util.motion.SetpointGenerator;
import frc.robot.vision.AimingParameters;

public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private WPI_TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  // Drive is plummed to default to high gear
  private boolean mHighGear = true;

  public static final int DEFAULT_ACCEL = 750;
  public static final int DEFAULT_CRUISE_VELOCITY = 900;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  private PeriodicDriveData mPeriodicDriveData = new PeriodicDriveData();
  private Logger mDriveLogger;

  public static class PeriodicDriveData {
    // INPUTS
    public double timestamp;
    public double left_voltage;
    public double right_voltage;
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public Rotation2d gyro_heading = Rotation2d.identity();
    public Pose2d error = Pose2d.identity();

    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_feedforward;
    public double right_feedforward;
    public double left_old = 0;
    public double right_old = 0;
    public boolean isQuickturning = false;
  }

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  public int feetToTicks(double feet) {
    long roundedVal = Math.round(feet * Constants.TICKS_PER_FOOT);
    if (roundedVal > Integer.MAX_VALUE) {
      mDriveLogger.warn(
          "Integer overflow when converting feet to ticks! Something is likely VERY WRONG!");
    }

    return (int) roundedVal;
  }

  private Drive() {
    mDriveLogger = new Logger("drive");

    mLeftMaster = new WPI_TalonSRX(Config.getInstance().getInt(Key.DRIVE__LEFT_BACK_PORT));
    // configureSpark(mLeftMaster, true, true);

    mLeftSlave = new WPI_TalonSRX(Config.getInstance().getInt(Key.DRIVE__LEFT_FRONT_PORT));
    // configureSpark(mLeftSlave, true, false);

    mRightMaster = new WPI_TalonSRX(Config.getInstance().getInt(Key.DRIVE__RIGHT_BACK_PORT));
    // configureSpark(mRightMaster, false, true);

    mRightSlave = new WPI_TalonSRX(Config.getInstance().getInt(Key.DRIVE__RIGHT_FRONT_PORT));
    // configureSpark(mRightSlave, false, false);

    configTalon(mLeftMaster);
    mLeftSlave.setNeutralMode(NeutralMode.Brake);

    configTalon(mRightMaster);
    mRightSlave.setNeutralMode(NeutralMode.Brake);

    // Configure slave Talons to follow masters
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    // TODO: figure out what this does and make it work
    setOpenLoop(DriveSignal.NEUTRAL);
  }

  private void configTalon(WPI_TalonSRX talon) {
    talon.configFactoryDefault();
    talon.configNominalOutputForward(0., 0);
    talon.configNominalOutputReverse(0., 0);
    talon.configPeakOutputForward(1, 0);
    talon.configPeakOutputReverse(-1, 0);
    talon.configOpenloopRamp(0);
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talon.setSensorPhase(true);
    talon.setNeutralMode(NeutralMode.Brake);

    // Configure Talon gains
    double P, I, D;

    P = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_P);
    I = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_I);
    D = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_D);

    mDriveLogger.info("PID values: " + P + ", " + I + ", " + D);

    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);
    talon.config_kF(0, 0);
    // talon.configClosedloopRamp(0.5, 0);
    // talon.configClosedLoopPeakOutput(0, 0.5);
    // talon.configPeakCurrentDuration(3000);
    // talon.configPeakCurrentLimit(30);
    // talon.configContinuousCurrentLimit(15);
    talon.configClosedLoopPeriod(0, 10);

    talon.configMotionCruiseVelocity(900);
    talon.configMotionAcceleration(750);

    // talon.configClosedloopRamp(1);
  }

  public void resetCruiseAndAccel() {
    setCruiseAndAcceleration(DEFAULT_CRUISE_VELOCITY, DEFAULT_ACCEL);
  }

  public void setCruiseAndAcceleration(int cruise, int accel) {
    mLeftMaster.configMotionCruiseVelocity(cruise);
    mRightMaster.configMotionCruiseVelocity(cruise);

    mLeftMaster.configMotionAcceleration(accel);
    mRightMaster.configMotionAcceleration(accel);
  }

  public void configP(double p) {
    mLeftMaster.config_kP(0, p);
    mRightMaster.config_kP(0, p);
  }
  
  public void configI(double i) {
    mLeftMaster.config_kI(0, i);
    mRightMaster.config_kI(0, i);
  }

  public void configD(double d) {
    mLeftMaster.config_kD(0, d);
    mRightMaster.config_kD(0, d);
  }

  public void resetPID() {
    double P, I, D;

    P = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_P);
    I = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_I);
    D = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_D);

    configP(P);
    configI(I);
    configD(D);
  }

  public void zeroSensors() {
    zeroEncoders();
  }

  public void setMotionMagicTarget(int left, int right) {
    mLeftMaster.set(ControlMode.MotionMagic, left);
    mRightMaster.set(ControlMode.MotionMagic, -right);
  }

  public void setSimplePIDTarget(int left, int right) {
    mLeftMaster.set(ControlMode.Position, left);
    mRightMaster.set(ControlMode.Position, -right);
  }

  /** Configure talons for open loop control */
  public synchronized void setOpenLoop(DriveSignal signal) {
    // A lot of the space in this function is taken up by local copies of stuff
    double accelSpeed = Config.getInstance().getDouble(Key.DRIVE__FORWARD_ACCEL_RAMP_TIME_SECONDS);
    double brakeSpeed = Config.getInstance().getDouble(Key.DRIVE__REVERSE_BRAKE_RAMP_TIME_SECONDS);

    // are we quickturning?
    boolean quickturn = mPeriodicDriveData.isQuickturning;

    // Segments are started by the variables they will need
    boolean leftStationary = false;
    boolean rightStationary = false;

    if (mLeftMaster.getSupplyCurrent() == 0) {
      leftStationary = true;
    }

    if (mRightMaster.getSupplyCurrent() == 0) {
      rightStationary = true;
    }

    // Splitting up var definitions to their unique sections makes the code more readable
    boolean stationary = false;

    if (leftStationary && rightStationary) {
      stationary = true;
    }
    
    // Don't know why this is here but I'm not gonna remove it
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      // setBrakeMode(true);
      mDriveLogger.verbose("switching to open loop " + signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    // Cache our signals for more readable code. right is backwards because reasons out of our control
    double leftOutput = signal.getLeft();
    double rightOutput = signal.getRight();

    // Ramping is calculated through a series of "abstractions", calculating the acceleration directions
    // of hierarchical components in the drivetrain.
    boolean leftAcceleratingForward = false;
    boolean leftAcceleratingBackwards = false;

    boolean rightAcceleratingForward = false;
    boolean rightAcceleratingBackwards = false;

    if (leftOutput > mPeriodicDriveData.left_old) {
      leftAcceleratingForward = true;
    } else if (leftOutput < mPeriodicDriveData.left_old) {
      leftAcceleratingBackwards = true;
    }

    if (rightOutput > mPeriodicDriveData.right_old) {
      rightAcceleratingForward = true;
    } else if (rightOutput < mPeriodicDriveData.right_old) {
      rightAcceleratingBackwards = true;
    }

    // Whether our velocity is increasing or decreasing
    boolean acceleratingForward = false;
    boolean acceleratingBackwards = false;

    if (leftAcceleratingForward && rightAcceleratingForward) {
      acceleratingForward = true;
    } else if (leftAcceleratingBackwards && rightAcceleratingBackwards) {
      acceleratingBackwards = true;
    }

    // Whether we are going forwards or in reverse
    boolean velocityForwards = false;
    boolean velocityReverse = false;

    if (leftOutput > 0) {
      velocityForwards = true;
    } else if (leftOutput < 0) {
      velocityReverse = true;
    }

    // This is where the actual accel limiting logic begins
    if (velocityForwards) {
      if (acceleratingForward) {
        setOpenloopRamp(accelSpeed);
      }
    } else if (velocityReverse) {
      if (acceleratingForward) {
        setOpenloopRamp(brakeSpeed);
      }
    } else if (stationary) {
      setOpenloopRamp(0);
    } else if (quickturn) {
      setOpenloopRamp(0);
    }

    // cache our olds after we've used them to make them actually "olds"
    mPeriodicDriveData.left_old = leftOutput;
    mPeriodicDriveData.right_old = rightOutput;

    // then we set our master talons, remembering that the physical right of the drivetrain is backwards
    mLeftMaster.set(ControlMode.PercentOutput, leftOutput);
    mRightMaster.set(ControlMode.PercentOutput, rightOutput * -1);
  }

  // Used for arcade turning during auto
  public void setSimplePercentOutput(DriveSignal signal) {
    setOpenloopRamp(0); // Just in case
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
    wheel = wheel * -1; // invert wheel

    // TODO: Extract this "epsilonEquals" pattern into a "handleDeadband" method
    // If we're not pushing forward on the throttle, automatically enable quickturn so that we
    // don't have to
    // explicitly enable it before turning.
    if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
      throttle = 0.0;
      quickTurn = true;
    }

    // This is just a convoluted way to do a deadband.
    if (Util.epsilonEquals(wheel, 0.0, 0.020)) {
      wheel = 0.0;
    }

    if (wheel != 0 && quickTurn) {
      mPeriodicDriveData.isQuickturning = true;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    // We pass 0 for dy because we use a differential drive and can't strafe.
    // The wheel here is a constant curvature rather than an actual heading. This is what makes
    // the drive cheesy.
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));

    // Either the bigger of the two drive signals or 1, whichever is bigger.
    double scaling_factor =
        Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    if (quickTurn) {
      setOpenLoop(
          new DriveSignal(
              (signal.getLeft() / scaling_factor) / 1.5,
              (signal.getRight() / scaling_factor) / 1.5));
    } else {
      setOpenLoop(
          new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }
  }

  /*
      Auto Steer functionality
      passed in parameters to the goal to aim at
      allows driver to control throttle
      this will be called with the ball as a target
  */
  // public synchronized void autoSteerBall(double throttle, AimingParameters aim_params) {
  //   double timestamp = Timer.getFPGATimestamp();
  //   final double kAutosteerAlignmentPointOffset = 15.0; //
  //   /*
  //   setOpenLoop(Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curvature * throttle *
  // (reverse ? -1.0 : 1.0))));
  //   setBrakeMode(true);
  //   */

  // }


  //Auto Steer functionality to the goal
  //driver only controls the throttle
  //'we may want to set a speed floor/speed minimum'
  public synchronized void autoSteerFeederStation(double throttle, double angle){
    //double heading_error_rad = vehicle_to_alignment_point_bearing.getRadians();
    double radians = (0.0174533) * angle;
    double heading_error_rad = radians;
    final double kAutosteerKp = 0.05;
    boolean towards_goal = true;
    boolean reverse = false;
    double curvature = (towards_goal ? 1.0 : 0.0) * heading_error_rad * kAutosteerKp;
    setOpenLoop(Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curvature * throttle * (reverse ? -1.0 : 1.0))));
    //setBrakeMode(true);
  }


  public void setOpenloopRamp(double speed) {
    mLeftMaster.configOpenloopRamp(speed);
    mRightMaster.configOpenloopRamp(speed);
  }

  /**
   * WPILib's arcade drive. We need this for auto turning because it allows us to set a rotation
   * speed. Note that the deadband functionality has been removed, since we don't have to worry
   * about driver error during auto. If we were using WPILib's {@link
   * edu.wpi.first.wpilibj.drive.DifferentialDrive DifferentialDrive} we wouldn't need to copy this
   * over. The output is also clamped so that we don't lose control.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeHack(double xSpeed, double zRotation, boolean squareInputs) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    final double max = 0.5;

    setSimplePercentOutput(
        new DriveSignal(
            MathUtil.clamp(leftMotorOutput, -max, max),
            MathUtil.clamp(rightMotorOutput, -max, max)));
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  public synchronized int getLeftEncoderDistance() {
    return mLeftMaster.getSelectedSensorPosition();
  }

  public synchronized int getRightEncoderDistance() {
    return mRightMaster.getSelectedSensorPosition();
  }

  public synchronized Rotation2d getRotation() {
    return null;
  }

  public void zeroEncoders() {
    mLeftMaster.getSensorCollection().setQuadraturePosition(0, 250);
    mRightMaster.getSensorCollection().setQuadraturePosition(0, 250);
  }

  // Used only in TEST mode
  public void setOutputLeftBack(double output) {
    mLeftMaster.set(ControlMode.PercentOutput, output);
  }

  // Used only in TEST mode
  public void setOutputLeftFront(double output) {
    mLeftSlave.set(ControlMode.PercentOutput, output);
  }

  // Used only in TEST mode
  public void setOutputRightBack(double output) {
    mRightMaster.set(ControlMode.PercentOutput, output);
  }

  public synchronized Rotation2d getHeading() {
    return mPeriodicDriveData.gyro_heading;
  }

  // Used only in TEST mode
  public void setOutputRightFront(double output) {
    mRightSlave.set(ControlMode.PercentOutput, output);
  }

  public double getLeftLinearVelocity() {
    return 0;
  }

  public double getRightLinearVelocity() {
    return 0;
  }
}
