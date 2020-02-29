package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.Key;

/** Add your docs here. */
public class Storage extends Subsystem {

  private final int ROLLER_BOTTOM_PORT =
      Config.getInstance().getInt(Key.STORAGE__BOTTOM_ROLLER);
  private final int ROLLER_TOP_PORT = Config.getInstance().getInt(Key.STORAGE__TOP_ROLLER);

  private final int STORAGE_CAPICTY = 5;

  private final double STORE_SPEED =
      Config.getInstance().getDouble(Key.STORAGE__ROLLER_STORE_SPEED);
  private final double BOTTOM_SPEED_FACTOR =
      Config.getInstance().getDouble(Key.STORAGE__ROLLER_BOTTOM_SPEED_FACTOR);
  private final double TEST_SPEED_FACTOR =
      Config.getInstance().getDouble(Key.STORAGE__ROLLER_SPEED_FACTOR);
  private final double EJECT_SPEED =
      Config.getInstance().getDouble(Key.STORAGE__ROLLER_EJECT_SPEED);
  private final double BALL_DISTANCE_IN_ENCODER_TICKS;

  private final int INTAKE_SENSOR_PORT = 0;

  private DigitalInput mIntakeSensor;

  private final WPI_TalonSRX mBottomRoller;
  private final WPI_TalonSRX mTopRoller;

  private int mBallCount = 0;
  

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  public int getCapacity(){
    return STORAGE_CAPICTY;
  }

  private Storage() {
    mBottomRoller = new WPI_TalonSRX(ROLLER_BOTTOM_PORT);
    mTopRoller = new WPI_TalonSRX(ROLLER_TOP_PORT);

    mBottomRoller.configFactoryDefault();
    mTopRoller.configFactoryDefault();

    mTopRoller.setNeutralMode(NeutralMode.Brake);
    mBottomRoller.setNeutralMode(NeutralMode.Brake);
    
    mBottomRoller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    mIntakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);
    updateEncoderPosition();
    
    if (Robot.getIsPracticeBot()) {
      BALL_DISTANCE_IN_ENCODER_TICKS = Config.getInstance().getDouble(Key.STORAGE__BALL_DISTANCE_IN_ENCODER_TICKS_PRACTICE);
      mBottomRoller.setSensorPhase(false);
    } else {
      BALL_DISTANCE_IN_ENCODER_TICKS = Config.getInstance().getDouble(Key.STORAGE__BALL_DISTANCE_IN_ENCODER_TICKS_PRODUCTION);
      mBottomRoller.setSensorPhase(true);
    }
  }

  public synchronized int getEncoder() {
    // return the negative position that the talon gets us because it's hooked up backwards
    // this will return positive values
    return mBottomRoller.getSelectedSensorPosition();
  }

  public synchronized void updateEncoderPosition() {
    mBottomRoller.setSelectedSensorPosition(0);
    // mStartingEncoderPosition = getEncoder();
    System.out.println("RESETTING");
  }

  public synchronized boolean getIntakeSensor() {
    return mIntakeSensor.get();
  }

  public synchronized boolean isBallDetected() {
    return getIntakeSensor();
  }

  public synchronized boolean isBallStored() {
    System.out.println(getEncoder());
    // this allows us to fit a 5th ball
    if (sInstance.getBallCount() == 4) {
      return true;
    }

    int encoderDistance = getEncoder();
    SmartDashboard.putNumber("Encoder Distance", encoderDistance);
    SmartDashboard.putNumber("Encoder Distance Raw", getEncoder());

    // if we've hit our encoder distance target
    if (encoderDistance >= BALL_DISTANCE_IN_ENCODER_TICKS) {
      return true;
    } else {
      return false;
    }
  }

  public synchronized void barf() {
    mBottomRoller.set(ControlMode.PercentOutput, -(EJECT_SPEED * BOTTOM_SPEED_FACTOR));
    mTopRoller.set(ControlMode.PercentOutput, -(EJECT_SPEED));
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

  public synchronized boolean isEmpty() {
    return mBallCount == 0;
  }

  public synchronized boolean isFull() {
    return mBallCount == STORAGE_CAPICTY;
  }

  public synchronized void storeBall() {
    mBottomRoller.set(ControlMode.PercentOutput, STORE_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  public synchronized void emptyBalls() {
    mBallCount = 0;
  }

  /** Stops the roller. */
  public synchronized void stop() {
    mBottomRoller.set(ControlMode.PercentOutput, 0);
    mTopRoller.set(ControlMode.PercentOutput, 0);
  }

  public synchronized void ejectBall() {
    mBottomRoller.set(ControlMode.PercentOutput, EJECT_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  public synchronized void setBottomOutput(double output) {
    mBottomRoller.set(ControlMode.PercentOutput, output * BOTTOM_SPEED_FACTOR * TEST_SPEED_FACTOR);
  }

  public synchronized void setTopOutput(double output) {
    mTopRoller.set(ControlMode.PercentOutput, output * TEST_SPEED_FACTOR);
  }

  public synchronized int getBallCount() {
    return mBallCount;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
