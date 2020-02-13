package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Config;
import frc.robot.Logger;
import frc.robot.Config.Key;

/** Add your docs here. */
public class Storage extends Subsystem {

  private static final int ROLLER_BOTTOM_PORT = Config.getInstance().getInt(Key.STORAGE__BOTTOM_ROLLER);
  private static final int ROLLER_TOP_PORT = Config.getInstance().getInt(Key.STORAGE__TOP_ROLLER);

  private static final int INTAKE_SENSOR_PORT =
      Config.getInstance().getInt(Key.INTAKE__SENSOR);

  private static final int STORAGE_CAPICTY = 4;

  private static final double STORE_SPEED = Config.getInstance().getInt(Key.STORAGE__ROLLER_STORE_SPEED);
  private static final double BOTTOM_SPEED_FACTOR = Config.getInstance().getDouble(Key.STORAGE__ROLLER_BOTTOM_SPEED_FACTOR);
  private static final double SPEED_FACTOR = Config.getInstance().getDouble(Key.STORAGE__ROLLER_SPEED_FACTOR);
  private static final double EJECT_SPEED = Config.getInstance().getInt(Key.STORAGE__ROLLER_EJECT_SPEED);

  private WPI_TalonSRX mBottomRoller;
  private WPI_TalonSRX mTopRoller;
  private DigitalInput mIntakeSensor;

  private int mBallCount = 0;

  private boolean mWasLineBroke = false;

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
  }

  private boolean isLineBroken() {
    return mIntakeSensor.get();
  }

  public boolean isBallStored() {
    // if the last time we checked the line was broken and now it isn't, we've just stored a ball
    if (mWasLineBroke && !isLineBroken()) {
      mWasLineBroke = isLineBroken();
      return true;
    // otherwise, we need to wait until the sensor line stops being broken
    } else {
      mWasLineBroke = isLineBroken();
      return false;
    }
  }

  public void barf(){
    mBottomRoller.set(ControlMode.PercentOutput, -(STORE_SPEED * BOTTOM_SPEED_FACTOR));
    mTopRoller.set(ControlMode.PercentOutput, -(STORE_SPEED));
  }

  public synchronized void preloadBalls(int ballCount) {
    mBallCount = ballCount;
  }

  public synchronized void addBall() {
    if (mBallCount < STORAGE_CAPICTY) {
      mBallCount++;
    }
  }

  public synchronized void removeBall() {
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
    mBottomRoller.set(ControlMode.PercentOutput, STORE_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mBottomRoller.set(ControlMode.PercentOutput, 0);
    mTopRoller.set(ControlMode.PercentOutput, 0);
  }

  public void ejectBall() {
    mBottomRoller.set(ControlMode.PercentOutput, EJECT_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  public void setBottomOutput(double output) {
    mBottomRoller.set(ControlMode.PercentOutput, output * BOTTOM_SPEED_FACTOR * SPEED_FACTOR);
  }

  public void setTopOutput(double output) {
    mTopRoller.set(ControlMode.PercentOutput, output * SPEED_FACTOR);
  }

  public int getBallCount() {
    return mBallCount;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
