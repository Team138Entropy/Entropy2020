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
    public boolean wasReversing = false;
    public boolean quickturn = false;
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
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      // setBrakeMode(true);
      mDriveLogger.verbose("switching to open loop " + signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    signal.PrintLog();

    // Very first step is to update our negative magnitude checker
    if (signal.getLeft() < 0 && signal.getRight() < 0) {
      mPeriodicDriveData.wasReversing = true;
    } else if (signal.getLeft() > 0 && signal.getRight() > 0) {
      mPeriodicDriveData.wasReversing = false;
    }

    // If our acceleration is positive (going away from where we were last time)
    // Remember that right has to be flipped down below so the bracket is the other way 'round
    // mDriveLogger.log("Left: " + signal.getLeft() + " + " + mPeriodicDriveData.left_old);
    // mDriveLogger.log("Right: " + signal.getRight() + " + " + mPeriodicDriveData.right_old);
    if ((Math.abs(signal.getLeft()) > mPeriodicDriveData.left_old)
        && (Math.abs(signal.getRight()) > mPeriodicDriveData.right_old)
        && !mPeriodicDriveData.quickturn) {
      // Tell the talons to be significantly less epic
      setOpenloopRamp(Config.getInstance().getDouble(Key.DRIVE__ACCEL_RAMP_TIME_SECONDS));
    }
    // If the opposite is true, e.g. our velocity is decreasing, let us stop as fast as we want.
    // Note that this
    // "inverse case" is here because, if it wasn't, acceleration would only be capped while jerk is
    // positive.
    else if (Math.abs(signal.getLeft()) < mPeriodicDriveData.left_old
        && Math.abs(signal.getRight()) < mPeriodicDriveData.right_old
        && !mPeriodicDriveData.quickturn) {
      // Actually don't stop as fast as we can, first check if we are going backwards by comparing
      // the
      // magnitudes of left and right drive signals
      if (mPeriodicDriveData.wasReversing = true) {
        setOpenloopRamp(Config.getInstance().getDouble(Key.DRIVE__BACK_SLOW_RAMP_TIME_SECONDS));
      } else {
        // If we are going forwards, we can stop as fast as we want.
        setOpenloopRamp(0);
      }
    }
    // In the case that our joystick value is zero, disable our shit as it was last time, nothing
    // happens to our ramp
    else if (signal.getLeft() == 0 && signal.getRight() == 0 && !mPeriodicDriveData.wasReversing) {
      setOpenloopRamp(0);
    }
    // At this point this is just getting ridiculous. Hopefully this is self-evident.
    else if (mPeriodicDriveData.quickturn) {
      setOpenloopRamp(0);
    }

    // Olds are cached as absolute to be useful above
    mPeriodicDriveData.left_old = Math.abs(signal.getLeft());
    mPeriodicDriveData.right_old = Math.abs(signal.getRight());

    // then we set our master talons
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  // Used for arcade turning during auto
  public void setSimplePercentOutput(DriveSignal signal) {
    setOpenloopRamp(0); // Just in case
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
    wheel = wheel * -1; // invert wheel

    mPeriodicDriveData.quickturn = quickTurn;

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

  public void setOpenloopRamp(double speed) {
    // mDriveLogger.log("setting ramp to " + speed);
    mLeftMaster.configOpenloopRamp(speed);
    mRightMaster.configOpenloopRamp(speed);
  }

  /**
   * WPILib's arcade drive. We need this for auto turning because it allows us to set a rotation
   * speed. Note that the deadband functionality has been removed, since we don't have to worry
   * about driver error during auto. If we were using WPILib's {@link
   * edu.wpi.first.wpilibj.drive.DifferentialDrive DifferentialDrive} we wouldn't need to copy this
   * over.
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

    setSimplePercentOutput(
        new DriveSignal(
            MathUtil.clamp(leftMotorOutput, -1.0, 1.0),
            MathUtil.clamp(rightMotorOutput, -1.0, 1.0)));
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  public synchronized int getLeftEncoderDistance() {
    return mLeftMaster.getSelectedSensorPosition(0);
  }

  public synchronized int getRightEncoderDistance() {
    return -mRightMaster.getSelectedSensorPosition(0);
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
