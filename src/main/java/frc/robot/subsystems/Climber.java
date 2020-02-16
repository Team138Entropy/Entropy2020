package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Config;


public class Climber extends Subsystem {

  /** Aggregation */
  private static Climber sInstance;
  private WPI_TalonSRX mMotor;

  //TODO: Tune climber config values
  /** Speed of motor in % of maximum output*/
  private final double EXTEND_SPEED = Config.getInstance().getDouble(Config.Key.CLIMBER_EXTEND_SPEED);
  private final double RETRACT_SPEED = Config.getInstance().getDouble(Config.Key.CLIMBER_RETRACT_SPEED);

  /** Overcurrent values copied from intake, not sure what they do */
  private final double OVERCURRENT_THRESHOLD = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_THRESHOLD);
  private final double OVERCURRENT_MIN_OCCURENCES = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_MIN_OCCURENCES);

  //TODO: Check if this is the right roller port
  private final double PORT_NUMBER = Config.getInstance().getInt(Config.Key.CLIMBER_ROLLER);

  /** For measuring how long the motor's been running for */
  private double mOverCurrentCountdown;
  private int mOverCurrentCount;

  private double mCurrent;


  public Climber() {
    mMotor = new WPI_TalonSRX(0);
    mOverCurrentCountdown = Config.getInstance().getDouble(Config.Key.CLIMBER_OVERCURRENT_COUNTDOWN_LENGTH);
    mOverCurrentCount = 0;
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void extend() {
    mMotor.set(ControlMode.PercentOutput, EXTEND_SPEED);
  }

  public void retract() {
    mMotor.set(ControlMode.PercentOutput, RETRACT_SPEED);
  }

  public void stop() {
    mMotor.stopMotor();
  }

  public boolean checkOvercurrent() {
    mCurrent = mMotor.getSupplyCurrent();
    if (mOverCurrentCountdown > 0){
      mOverCurrentCountdown --;
    } else if (mCurrent > OVERCURRENT_THRESHOLD) {
      mOverCurrentCount ++;
    }
    return mOverCurrentCountdown > 0 && mCurrent > OVERCURRENT_THRESHOLD;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
