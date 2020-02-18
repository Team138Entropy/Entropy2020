package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Config;
import frc.robot.Logger;


public class Climber extends Subsystem {

  //TODO: Tune climber config values
  /** Speed of motor in % of maximum output*/
  private final double EXTEND_SPEED = Config.getInstance().getDouble(Config.Key.CLIMBER_EXTEND_SPEED);
  private final double RETRACT_SPEED = Config.getInstance().getDouble(Config.Key.CLIMBER_RETRACT_SPEED);

  private final double OVERCURRENT_THRESHOLD = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_THRESHOLD);
  private final double OVERCURRENT_MIN_OCCURENCES = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_MIN_OCCURENCES);
  private final double OVERCURRENT_COUNTDOWN_LENGTH = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_COUNTDOWN_LENGTH);

  //TODO: Check if this is the right roller port
  private final int PORT_NUMBER = Config.getInstance().getInt(Config.Key.CLIMBER_ROLLER);

  /** Aggregation */
  private static Climber sInstance;
  private WPI_TalonSRX mMotor;
  private Logger mLogger;

  /** For checking overcurrent */
  private double mOverCurrentCountdown;
  private int mOverCurrentCount;

  private Climber() {
    mMotor = new WPI_TalonSRX(PORT_NUMBER);
    mOverCurrentCountdown = OVERCURRENT_COUNTDOWN_LENGTH;
    mOverCurrentCount = 0;
    mLogger = new Logger("climber");
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void extend() {
    mLogger.verbose("Extending climber at " + EXTEND_SPEED + "% speed");
    mMotor.set(ControlMode.PercentOutput, EXTEND_SPEED);
  }

  public void retract() {
    mLogger.verbose("Retracting the climber at " + RETRACT_SPEED + "% speed");
    mMotor.set(ControlMode.PercentOutput, RETRACT_SPEED);
  }

  public void stop() {
    mLogger.verbose("Stopping the climber");
    mMotor.stopMotor();
  }

  public boolean checkOvercurrent() {
    mLogger.verbose("Input current: " + mMotor.getSupplyCurrent() + ", Output current: " + mMotor.getStatorCurrent());
    if (mOverCurrentCountdown > 0){
      mOverCurrentCountdown --;
      return false;
    }
    double current = mMotor.getSupplyCurrent();
    if (current > OVERCURRENT_THRESHOLD) {
      mOverCurrentCount++;
      mLogger.log("Debounce overcurrent " + mOverCurrentCount);
    } else {
      resetOvercurrentCount();
    }
    if (mOverCurrentCount > OVERCURRENT_MIN_OCCURENCES) {
      mLogger.log("Overcurrent!");
      resetOvercurrentCount();
      return true;
    } else {
      return false;
    }
  }

  private void resetOvercurrentCountdown() {
    mOverCurrentCountdown = OVERCURRENT_COUNTDOWN_LENGTH;
  }

  private void resetOvercurrentCount() {
    mOverCurrentCount = 0;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
