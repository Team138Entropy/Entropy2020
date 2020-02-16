package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Logger;

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

  public void barf() {
    mRoller.set(ControlMode.PercentOutput, -ROLLER_SPEED);
  }

  public void resetOvercurrentCooldown() {
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

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Intake Current Countdown", mOverCurrentCountdown);
    double current = mRoller.getStatorCurrent();    
    SmartDashboard.putNumber("Intake Current", current);
    SmartDashboard.putBoolean("Intake Spinning Up", false);
    SmartDashboard.putBoolean("Intake Overcurrent", false);
    SmartDashboard.putBoolean("Intake Overcurrent Debounced", false);
  }

  public boolean isBallDetected() {
    updateSmartDashboard();
      
    

    // this counts down to account for the fact that the roller will overcurrent when spinning up
    if (mOverCurrentCountdown > 0) {
      SmartDashboard.putBoolean("Intake Spinning Up", true);
      mOverCurrentCountdown--;
      return false;
    }

    double current = mRoller.getStatorCurrent();

    // if our current is at the threshold that's considered overcurrent...
    if (current >= Config.getInstance().getDouble(Key.INTAKE__OVERCURRENT_THRESHOLD)) {
      // ...increment a counter
      mOverCurrentCount++;
      mLogger.log("debounce overcurrent " + mOverCurrentCount);
      SmartDashboard.putBoolean("Intake Overcurrent", true);
    } else {
      // if not, reset it
      mOverCurrentCount = 0;
    }

    // if we've been at overcurrent for the last few occurences, we can return true
    if (mOverCurrentCount
        >= Config.getInstance().getDouble(Key.INTAKE__OVERCURRENT_MIN_OCCURENCES)) {
      mLogger.log("Overcurrent!");
      mOverCurrentCount = 0;
      SmartDashboard.putBoolean("Intake Overcurrent Debounced", true);
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
