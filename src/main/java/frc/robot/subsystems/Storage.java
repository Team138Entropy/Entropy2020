package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/** 
 * Storage System is made up of two motors
 * 
 */
public class Storage extends Subsystem {

  private final WPI_TalonSRX mLowerRoller;
  private final WPI_TalonSRX mUpperRoller;
  private final DigitalInput mIntakeSensor;

  private int mBallCount = 0;

  private boolean IsOn = false;

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  private Storage() {
    mLowerRoller = new WPI_TalonSRX(Constants.kStorageTalon1Port);
    mUpperRoller = new WPI_TalonSRX(Constants.kStorageTalon2Port);
    mIntakeSensor = new DigitalInput(Constants.kStorageBeamSensorPort);

    //The way the rollers are setup, both motors need to run to work
    mLowerRoller.follow(mUpperRoller);
  }

  public synchronized boolean IsOn(){
      return this.IsOn;
  }

  //Simple Enable the 
  public synchronized void Enable(){
    IsOn = true;
  }

  public synchronized void Disable(){
      IsOn = false;
  }

  public synchronized boolean isBallDetected() {
    return (mIntakeSensor.get());
  }

  public boolean isBallStored() {
    return (!mIntakeSensor.get());
  }

  public void preloadBalls(int ballCount) {
    mBallCount = ballCount;
  }

  public void addBall() {
    if (mBallCount < Constants.kStorageCapacity) {
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
    return mBallCount == Constants.kStorageCapacity;
  }

  public void storeBall() {
    //mRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  /** Stops the roller. */
  public synchronized void stop() {
    //mRoller.set(ControlMode.PercentOutput, 0);
  }

  public synchronized void ejectBall() {
  //  mRoller.set(ControlMode.PercentOutput, Constants.kStorageCapacity);
  }

  public int getBallCount() {
    return mBallCount;
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}