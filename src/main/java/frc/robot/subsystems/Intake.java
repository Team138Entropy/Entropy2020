package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Config;
import frc.robot.Logger;
import frc.robot.Config.Key;

/** Add your docs here. */
public class Intake extends Subsystem {
  private Logger mLogger = new Logger("intake");
  private int mOverCurrentCount = 0;

  // initial cooldown because our startup of the roller induces a countdown
  private int mOverCurrentCountdown = 30;

  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.INTAKE__ROLLER);

  // TODO: Tune these values
  private static final double ROLLER_SPEED =
      Config.getInstance().getDouble(Key.INTAKE__ROLLER_SPEED);

  private WPI_TalonSRX mRoller;

  private static Intake sInstance;

  public static synchronized Intake getInstance() {
    if (sInstance == null) {
      sInstance = new Intake();
    }
    return sInstance;
  }

  private Intake() {
    mRoller = new WPI_TalonSRX(ROLLER_PORT);
  }

  public void resetOvercurrentCooldown(){
    mOverCurrentCountdown = 30;
  }

  public void start() {
    mRoller.set(ControlMode.PercentOutput, ROLLER_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  // This is for TEST mode only
  public void setOutput(double output) {
    mRoller.set(ControlMode.PercentOutput, output);
  }
  
  private int counter = 0;
  public boolean isBallDetected(){
    mLogger.verbose("Input current: " + mRoller.getSupplyCurrent() + ", Output current: " + mRoller.getStatorCurrent());

    
    if(mOverCurrentCountdown > 0){
      mOverCurrentCountdown --;
      return false;
    }
    
    double current = mRoller.getSupplyCurrent();
    if (current >= Config.getInstance().getDouble(Key.INTAKE__OVERCURRENT_THRESHOLD)){
      mOverCurrentCount++;
      mLogger.log("debounce overcurrent " + mOverCurrentCount);
    }else{
      mOverCurrentCount = 0;
    }
    if (mOverCurrentCount >= Config.getInstance().getDouble(Key.INTAKE__OVERCURRENT_MIN_OCCURENCES)){
      mLogger.log("Overcurrent!");
      mOverCurrentCount = 0;
      return true;
    }else{
      return false;
    }
  }
  
  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
