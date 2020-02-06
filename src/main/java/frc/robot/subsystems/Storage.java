package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Config;
import frc.robot.Config.Key;

/** Add your docs here. */
public class Storage extends Subsystem {

  // TODO: Need to implement top roller
  private static final int ROLLER_BOTTOM_PORT = Config.getInstance().getInt(Key.STORAGE__BOTTOM_ROLLER);
  private static final int ROLLER_TOP_PORT = Config.getInstance().getInt(Key.STORAGE__TOP_ROLLER);

  private static final int INTAKE_SENSOR_PORT =
      Config.getInstance().getInt(Key.INTAKE__SENSOR);

  private static final int STORAGE_CAPICTY = 5;

  private static final double STORE_SPEED = Config.getInstance().getInt(Key.STORAGE__ROLLER_STORE_SPEED);
  private static final double EJECT_SPEED = Config.getInstance().getInt(Key.STORAGE__ROLLER_EJECT_SPEED);

  private WPI_TalonSRX mBottomRoller;
  private WPI_TalonSRX mTopRoller;
  private DigitalInput mIntakeSensor;

  private int mBallCount = 0;

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  private Storage() {
    mBottomRoller = new WPI_TalonSRX(ROLLER_BOTTOM_PORT);
    mTopRoller = new WPI_TalonSRX(ROLLER_TOP_PORT);
    mIntakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);
  }

  public void init() {
    mTopRoller.follow(mBottomRoller);
  }

  public boolean isBallDetected() {
    return (mIntakeSensor.get());
  }

  public boolean isBallStored() {
    return (!mIntakeSensor.get());
  }

  public void preloadBalls(int ballCount) {
    mBallCount = ballCount;
  }

  public void addBall() {
    if (mBallCount < STORAGE_CAPICTY) {
      mBallCount++;
    }
  }

  public void removeBall() {
    if (mBallCount > 0) {
      mBallCount--;
    }
  }

  public boolean isEmpty() {
    return mBallCount == 0;
  }

  public boolean isFull() {
    return mBallCount == STORAGE_CAPICTY;
  }

  public void storeBall() {
    // Top TALON is slaved to the bottom one, no need to command seperately. init() is only called in auto + teleop
    mBottomRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    // Top TALON is slaved to the bottom one, no need to command seperately. init() is only called in auto + teleop
    mBottomRoller.set(ControlMode.PercentOutput, 0);
  }

  public void ejectBall() {
    // Top TALON is slaved to the bottom one, no need to command seperately. init() is only called in auto + teleop
    mBottomRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  public int getBallCount() {
    return mBallCount;
  }

  public void setBottomOutput(double output) {
    mBottomRoller.set(ControlMode.PercentOutput, output);
  }

  public void setTopOutput(double output) {
    mTopRoller.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
