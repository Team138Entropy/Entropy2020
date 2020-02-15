package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Kinematics;
import frc.robot.Logger;
import frc.robot.util.*;
import frc.robot.util.geometry.*;
import frc.robot.vision.AimingParameters;

public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private WPI_TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  // Drive is plummed to default to high gear
  private boolean mHighGear = true;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  private PeriodicIO mPeriodicIO;
  private Logger mDriveLogger;

  public static class PeriodicIO {
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
    public double left_accel;
    public double right_accel;
    public double left_feedforward;
    public double right_feedforward;
  }

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
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
  }

  public void init() {
    mLeftMaster.configNominalOutputForward(0., 0);
    mLeftMaster.configNominalOutputReverse(0., 0);
    mLeftMaster.configPeakOutputForward(1, 0);
    mLeftMaster.configPeakOutputReverse(-1, 0);
    mLeftMaster.setNeutralMode(NeutralMode.Brake);
    mLeftMaster.setNeutralMode(NeutralMode.Brake);

    mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    mLeftMaster.setSensorPhase(true);
    mLeftMaster.configNominalOutputForward(0., 0);
    mLeftMaster.configNominalOutputReverse(-0., 0);
    mLeftMaster.configPeakOutputForward(1, 0);
    mLeftMaster.configPeakOutputReverse(-1, 0);
    mLeftMaster.setNeutralMode(NeutralMode.Brake);
    mLeftSlave.setNeutralMode(NeutralMode.Brake);

    // Configure Talon gains
    /*
    mLeftMaster.config_kF(0, Drive_Kf,0);
    mLeftMaster.config_kP(0, Drive_Kp,0);
    mLeftMaster.config_kI(0, Drive_Ki,0);
    mLeftMaster.config_kD(0, Drive_Kd,0);
    mRightMaster.config_kF(0, Drive_Kf,0);
    mRightMaster.config_kP(0, Drive_Kp,0);
    mRightMaster.config_kI(0, Drive_Ki,0);
    mRightMaster.config_kD(0, Drive_Kd,0);
          */

    // Configure slave Talons to follow masters
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    setOpenLoop(DriveSignal.NEUTRAL);
  }

  public void zeroSensors() {}

  /** Configure talons for open loop control */
  public synchronized void setOpenLoop(DriveSignal signal) {
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      // setBrakeMode(true);
      mDriveLogger.verbose("switching to open loop " + signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    signal.PrintLog();
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
    if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
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
    setOpenLoop(
        new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
  }

  /*
      Auto Steer functionality
      passed in parameters to the goal to aim at
      allows driver to control throttle
      this will be called with the ball as a target
  */
  public synchronized void autoSteerBall(double throttle, AimingParameters aim_params) {
    double timestamp = Timer.getFPGATimestamp();
    final double kAutosteerAlignmentPointOffset = 15.0; //
    /*
    setOpenLoop(Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curvature * throttle * (reverse ? -1.0 : 1.0))));
    setBrakeMode(true);
    */

  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  public synchronized double getLeftEncoderDistance() {
    return 0.0;
  }

  public synchronized double getRightEncoderDistance() {
    return 0.0;
  }

  public synchronized Rotation2d getRotation() {
    return null;
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
    return mPeriodicIO.gyro_heading;
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
