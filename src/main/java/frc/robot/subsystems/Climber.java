package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Config;
import frc.robot.Logger;


public class Climber extends Subsystem {
  private final int PORT_NUMBER = Config.getInstance().getInt(Config.Key.CLIMBER__MOTOR);

  //TODO: Tune these values
  private final double HEIGHT_IN_ENCODER_TICKS = Config.getInstance().getDouble(Config.Key.CLIMBER__HEIGHT_IN_ENCODER_TICKS);
  private final double RETRACTED_HEIGHT_IN_ENCODER_TICKS = Config.getInstance().getDouble(Config.Key.CLIMBER__RETRACTED_HEIGHT_IN_ENCODER_TICKS);
  private final double HOMING_SPEED_PERCENT = Config.getInstance().getDouble(Config.Key.CLIMBER__HOME_SPEED);

  /** Talon SRX/ Victor SPX will support multiple (cascaded) PID loops. For now we just want the primary one. */

  private final int PIDLoopIndex = Config.getInstance().getInt(Config.Key.CLIMBER__PID_LOOP_INDEX);

  /** Climber motion command timeout */
  private final int TimeoutMS = Config.getInstance().getInt(Config.Key.CLIMBER__TIMEOUT_MS);

  /** Servo loop gains */
  private final double mMotorKF = Config.getInstance().getDouble(Config.Key.CLIMBER__KF);
  private final double mMotorKP = Config.getInstance().getDouble(Config.Key.CLIMBER__KP);
  private final double mMotorKI = Config.getInstance().getDouble(Config.Key.CLIMBER__KI);
  private final double mMotorKD = Config.getInstance().getDouble(Config.Key.CLIMBER__KD);

  /** Aggregation */
  private static Climber sInstance;
  private WPI_TalonSRX mMotor;
  private Logger mLogger;
  private boolean mIsHoming;

  private Climber() {
    mMotor = new WPI_TalonSRX(PORT_NUMBER);
    mLogger = new Logger("climber");
    mIsHoming = false;
    init();
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void init() {

    /* Set the peak and nominal outputs, 12V means full */
    mMotor.configNominalOutputForward(0, TimeoutMS);
    mMotor.configNominalOutputReverse(0, TimeoutMS);
    mMotor.configPeakOutputForward(1, TimeoutMS);
    mMotor.configPeakOutputReverse(-1, TimeoutMS);

    mMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLoopIndex, TimeoutMS);
    mMotor.setSensorPhase(false);

    /* set the allowable closed-loop error,
     * Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    mMotor.configAllowableClosedloopError(0, PIDLoopIndex, TimeoutMS); /* always servo */

    mMotor.config_kF(PIDLoopIndex, mMotorKF, TimeoutMS);
    mMotor.config_kP(PIDLoopIndex, mMotorKP, TimeoutMS);
    mMotor.config_kI(PIDLoopIndex, mMotorKI, TimeoutMS);
    mMotor.config_kD(PIDLoopIndex, mMotorKD, TimeoutMS);

    // Set brake mode to hold at position
    mMotor.setNeutralMode(NeutralMode.Brake);

    // Integral control only applies when the error is small; this avoids integral windup
    mMotor.config_IntegralZone(0, 200, TimeoutMS);
  }

  public void extend() {
    mLogger.verbose("Extending climber to " + HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, HEIGHT_IN_ENCODER_TICKS);
  }

  public void retract() {
    mLogger.verbose("Retracting the climber to " + RETRACTED_HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, RETRACTED_HEIGHT_IN_ENCODER_TICKS);
  }

  /** Stops the climber */
  public void stop() {
    mLogger.verbose("Stopping the climber");
    if (mIsHoming) {
      stopHoming();
    }else {
      mMotor.stopMotor();
    }
  }

  /** Return true if extended */
  public boolean isExtended() {
    mMotor.getSelectedSensorPosition();
    return true;
  }

  /** Returns true if retracted */
  public boolean isRetracted() {
    return true;
  }

  /** Jogs the climber */
  public void jog(double speed) {
    mMotor.set(ControlMode.PercentOutput, speed);
  }

  /** Homes the climber */
  public void home() {
    mMotor.set(ControlMode.PercentOutput, HOMING_SPEED_PERCENT);
    mIsHoming = true;
  }

  /** Stops the  */
  private void stopHoming() {
    mMotor.stopMotor();
    setHome();
    mIsHoming = false;
  }

  /** Sets the encoder's home */
  public void setHome() {
    mLogger.verbose("Home set");
    mMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
