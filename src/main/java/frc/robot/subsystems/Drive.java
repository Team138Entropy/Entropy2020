package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Logger;
import frc.robot.util.*;
import frc.robot.util.geometry.*;
import frc.robot.vision.AimingParameters;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private final WPI_TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  //Drive Encoders
  private final Encoder mLeftEncoder, mRightEncoder;

  //Robot Gyro
  private final ADXRS450_Gyro mGyro;

  private Solenoid mGearSolenoid; // Gear Shifting Solenoid
  // private final Solenoid mShifter;

  // Drive is plummed to default to high gear
  private boolean mHighGear = true;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  private PeriodicIO mPeriodicIO;
  private Logger mDriveLogger;


  //Class containing information for periodic updates
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

  @Override
  public synchronized void readPeriodicInputs() {
      mPeriodicIO.timestamp = Timer.getFPGATimestamp();
      double prevLeftTicks = mPeriodicIO.left_position_ticks;
      double prevRightTicks = mPeriodicIO.right_position_ticks;

      mPeriodicIO.left_voltage = mLeftMaster.getMotorOutputVoltage() * mLeftMaster.getBusVoltage();
      mPeriodicIO.right_voltage = mRightMaster.getMotorOutputVoltage() * mRightMaster.getBusVoltage();

      mPeriodicIO.left_position_ticks = mLeftEncoder.get();
      mPeriodicIO.right_position_ticks = mRightEncoder.get();
      mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mGyro.getAngle());

      double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
              * Math.PI;
      mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

      double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
              * Math.PI;
      mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

      mPeriodicIO.left_velocity_ticks_per_100ms = (int) (mLeftEncoder.getRate()
              / (10 * mLeftEncoder.getDistancePerPulse()));
      mPeriodicIO.right_velocity_ticks_per_100ms = (int) (mRightEncoder.getRate()
              / (10 * mRightEncoder.getDistancePerPulse()));

  }

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  private Drive() {
    mDriveLogger = new Logger("drive");

    // Shifter Solenoid
    // mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

    mLeftMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterId);

    mLeftSlave = new WPI_TalonSRX(Constants.kLeftDriveSlaveId);

    mRightMaster = new WPI_TalonSRX(Constants.kRightDriveMasterId);

    mRightSlave = new WPI_TalonSRX(Constants.kRightDriveSlaveId);

    //Encoder Intialization
    //last argument is to reverse directions!
    mLeftEncoder = new Encoder(Constants.kLeftDriveEncoderPortA, Constants.kLeftDriveEncoderPortB, false);
    mRightEncoder = new Encoder(Constants.kRightDriveEncoderPortA, Constants.kRightDriveEncoderPortB, true);

    //Configure Distance Per Pulse
    mLeftEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
    mRightEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);

    //Intialize Gyro on RoboRio
    mGyro = new ADXRS450_Gyro();
    mGyro.reset();
    mGyro.calibrate();


    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_SOLENOID)) {
      mGearSolenoid = new Solenoid(Constants.kShifterSolenoidId);
    }
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


  //Clear Everything and start a new periodic IO container
  public synchronized void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
    mPeriodicIO = new PeriodicIO();
  }

  public synchronized void resetGyro(){
    //TODO: Find proper order of this
    mGyro.calibrate();
    mGyro.reset();
  }


  public void zeroSensors() {
    resetEncoders();
    resetGyro();
  }

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
      SwitchGears
      Toggles the current gear
  */
  public synchronized void switchGears() {
    if (Config.getInstance().getBoolean(Key.ROBOT__HAS_SOLENOID)) {
      mHighGear = !mHighGear;
      mGearSolenoid.set(mHighGear);
    }
  }

  /*
      SetGear
      TODO: This may be the incorrect polarity. Verfiy this.
  */
  public synchronized void setGear(boolean highGear) {
    mGearSolenoid.set(highGear);
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  


  private static double rotationsToInches(double rotations) {
      return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double rpmToInchesPerSecond(double rpm) {
      return rotationsToInches(rpm) / 60;
  }

  private static double inchesToRotations(double inches) {
      return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
      return inchesToRotations(inches_per_second) * 60;
  }

  private static double radiansPerSecondToTicksPer100ms(double rad_s) {
      return rad_s / (Math.PI * 2.0) * Constants.kDriveEncoderPPR / 10.0;
  }


    public double getLeftEncoderRotations() {
      return mPeriodicIO.left_position_ticks / Constants.kDriveEncoderPPR;
  }

  public double getRightEncoderRotations() {
      return mPeriodicIO.right_position_ticks / Constants.kDriveEncoderPPR;
  }

  public double getLeftEncoderDistance() {
      return rotationsToInches(getLeftEncoderRotations());
  }

  public double getRightEncoderDistance() {
      return rotationsToInches(getRightEncoderRotations());
  }

  public double getRightVelocityNativeUnits() {
      return mPeriodicIO.right_velocity_ticks_per_100ms;
  }

  public double getRightLinearVelocity() {
      return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
  }

  public double getLeftVelocityNativeUnits() {
      return mPeriodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
      return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
  }

  public double getLinearVelocity() {
      return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAverageDriveVelocityMagnitude() {
      return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
      return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
  }

  public synchronized Rotation2d getHeading() {
    return mPeriodicIO.gyro_heading;
}

  @Override
  public void stopSubsytem(){}

}
