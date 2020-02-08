package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Constants;;

/** Add your docs here. */
public class Storage extends Subsystem {

  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.STORAGE__ROLLER_PORT);
  private static final int INTAKE_SENSOR_PORT =
      Config.getInstance().getInt(Key.INTAKE__SENSOR_PORT);

  private static final int STORAGE_CAPICTY = 5;

  // TODO: Tune these values
  private static final double STORE_SPEED = Config.getInstance().getInt(Key.INTAKE__SENSOR_PORT);
  private static final double EJECT_SPEED = Config.getInstance().getInt(Key.STORAGE__ROLLER_PORT);

  private WPI_TalonSRX mLowerRoller;
  private WPI_TalonSRX mUpperRoller;
  private DigitalInput mIntakeSensor;
  private boolean Running = false;

  private double SpeedModifier = 1.0;
  

  private int mBallCount = 0;

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  private Storage() {
    //mRoller = new WPI_TalonSRX(ROLLER_PORT);
   // mIntakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);
   mLowerRoller = new WPI_TalonSRX(Constants.kStorageLowerTalon);
   mUpperRoller = new WPI_TalonSRX(Constants.kStorageUpperTalon);

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
    Running = true;
    mLowerRoller.set(ControlMode.PercentOutput, SpeedModifier * .4);
    mUpperRoller.set(ControlMode.PercentOutput, SpeedModifier * .5 );

  }

  /** Stops the roller. */
  public void stop() {
    Running = false;
    mLowerRoller.set(ControlMode.PercentOutput, 0);
    mUpperRoller.set(ControlMode.PercentOutput, 0);

  }

  public void invert(){
    if(SpeedModifier == -1){
      SpeedModifier = 1;
    }else{
      SpeedModifier = -1;
    }
  }

  public void ejectBall() {
   // mRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  

  public int getBallCount() {
    return mBallCount;
  }

  public synchronized boolean IsRunning(){
    return Running;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}
