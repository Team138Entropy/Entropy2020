package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Logger;

public class Storage extends Subsystem {


  private static final int STORAGE_CAPICTY = 4;


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
    mBottomRoller = new WPI_TalonSRX(Constants.kStorageLowerTalon);
    mTopRoller = new WPI_TalonSRX(Constants.kStorageUpperTalon);
    //mIntakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);
  }

  public void init() {
  }

  public boolean isBallStored() {
    return mIntakeSensor.get();
  }

  public boolean wasLineBroke(){
    // we have 2 modes to determine if we should stop the storage rollers
    // returning isBallStored() stops the rollers whenever the sensor is broken
    // the code below that line stops the rollers whenever the sensor is no longer broken
    return isBallStored();
    // if(mWasLineBroke && !isBallStored()){
    //   mWasLineBroke = false;
    //   return true;
    // }
    // if(isBallStored()){
    //   mWasLineBroke = true;
    // }
    // return false;
  }

  public void barf(){
    mBottomRoller.set(ControlMode.PercentOutput, -(Constants.kStorageRollerSpeed * Constants.kStorageRollerSpeed));
    mTopRoller.set(ControlMode.PercentOutput, -(Constants.kStorageRollerSpeed));
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
    mBottomRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerSpeed * Constants.kStorageRollerFactor);
    mTopRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerSpeed);
  }

  public void start(){
    mBottomRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerSpeed * Constants.kStorageRollerFactor);
    mTopRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerSpeed);
  }

  /** Stops the roller. */
  public void stop() {
    mBottomRoller.set(ControlMode.PercentOutput, 0);
    mTopRoller.set(ControlMode.PercentOutput, 0);
  }

  public void ejectBall() {
    mBottomRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerEject * Constants.kStorageRollerFactor);
    mTopRoller.set(ControlMode.PercentOutput, Constants.kStorageRollerEject);
  }

  public int getBallCount() {
    return mBallCount;
  }

  public void setBottomOutput(double output) {
    mBottomRoller.set(ControlMode.PercentOutput, output * Constants.kStorageRollerSpeed * Constants.kStorageRollerFactor);
  }

  public void setTopOutput(double output) {
    mTopRoller.set(ControlMode.PercentOutput, output * Constants.kStorageRollerSpeed);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}