package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Logger;
import frc.robot.Constants;

//starts the roller subsystem
public class Intake extends Subsystem {
  private Logger mLogger = new Logger("intake");


  private int mOverCurrentCount = 0;

  // initial cooldown because our startup of the roller induces a countdown
  private int mOverCurrentCountdown = 30;

  private WPI_TalonSRX mRoller;

  private static Intake sInstance;

  public static synchronized Intake getInstance() {
    if (sInstance == null) {
      sInstance = new Intake();
    }
    return sInstance;
  }

  private Intake() {
    mRoller = new WPI_TalonSRX(Constants.kIntakeRollerPort);
    mRoller.configFactoryDefault();
  }

  public void barf(){
    mRoller.set(ControlMode.PercentOutput, -Constants.kIntakeRollerSpeed);
  }

  public void resetOvercurrentCooldown(){
    mOverCurrentCountdown = 30;
  }

  //start the roller
  public void start() {
    mLogger.verbose("Running roller at " + Constants.kIntakeRollerSpeed);
    mRoller.set(ControlMode.PercentOutput, Constants.kIntakeRollerSpeed);
  }

  /** Stops the roller. */
  public void stop() {
    mLogger.verbose("Stopping roller ");
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  // This is for TEST mode only
  public void setOutput(double output) {
    mRoller.set(ControlMode.PercentOutput, output);
  }
  
  public boolean isBallDetected(){
    mLogger.verbose("Input current: " + mRoller.getSupplyCurrent() + ", Output current: " + mRoller.getStatorCurrent());

    // this counts down to account for the fact that the roller will overcurrent when spinning up
    if(mOverCurrentCountdown > 0){
      mOverCurrentCountdown --;
      return false;
    }
    
    double current = mRoller.getSupplyCurrent();

    // if our current is at the threshold that's considered overcurrent...
    if (current >= Constants.kIntakeCurrentThreshold){
      // ...increment a counter
      mOverCurrentCount++;
      mLogger.log("debounce overcurrent " + mOverCurrentCount);
    }else{
      // if not, reset it
      mOverCurrentCount = 0;
    }

    // if we've been at overcurrent for the last few occurences, we can return true
    if (mOverCurrentCount >=  Constants.kIntakeOverCurrentMinOccurances){
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

  @Override
  public void stopSubsytem() {}
}