package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import javax.naming.ldap.Control;

import frc.robot.Config;
import frc.robot.Logger;


public class Climber extends Subsystem {
  private final int PORT_NUMBER = Config.getInstance().getInt(Config.Key.CLIMBER__MOTOR);

  //TODO: Tune these values
  private final double HEIGHT_IN_ENCODER_TICKS = Config.getInstance().getDouble(Config.Key.CLIMBER__HEIGHT_IN_ENCODER_TICKS);
  private final double RETRACTED_HEIGHT_IN_ENCODER_TICKS = Config.getInstance().getDouble(Config.Key.CLIMBER__RETRACTED_HEIGHT_IN_ENCODER_TICKS);
  private final double HOMING_SPEED_PERCENT = -.2;

  // Talon SRX/ Victor SPX will support multiple (cascaded) PID loops
  // For now we just want the primary one.
  public static final int kClimberPIDLoopIndex = 0;

  // climber motion command timeout
  public static final int kClimberTimeoutMs = 10;

  // Servo Loop Gains
  double mClimberKf = 0.2;
  double mClimberKp = 5;
  double mClimberKi = 0.01;
  double mClimberKd = 10;

  /** Aggregation */
  private static Climber sInstance;
  private WPI_TalonSRX mMotor;
  private Logger mLogger;

  private Climber() {
    mMotor = new WPI_TalonSRX(PORT_NUMBER);
    mLogger = new Logger("climber");
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void init() {

    /* set the peak and nominal outputs, 12V means full */
    mMotor.configNominalOutputForward(0, kClimberTimeoutMs);
    mMotor.configNominalOutputReverse(0, kClimberTimeoutMs);
    mMotor.configPeakOutputForward(1, kClimberTimeoutMs);
    mMotor.configPeakOutputReverse(-1, kClimberTimeoutMs);

    mMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kClimberPIDLoopIndex, kClimberTimeoutMs);
    mMotor.setSensorPhase(false);

    /* set the allowable closed-loop error,
     * Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    mMotor.configAllowableClosedloopError(0, kClimberPIDLoopIndex, kClimberTimeoutMs); /* always servo */

    mMotor.config_kF(kClimberPIDLoopIndex, mClimberKf, kClimberTimeoutMs);
    mMotor.config_kP(kClimberPIDLoopIndex, mClimberKp, kClimberTimeoutMs);
    mMotor.config_kI(kClimberPIDLoopIndex, mClimberKi, kClimberTimeoutMs);
    mMotor.config_kD(kClimberPIDLoopIndex, mClimberKd, kClimberTimeoutMs);

    // Set brake mode to hold at position
    mMotor.setNeutralMode(NeutralMode.Brake);

    // Integral control only applies when the error is small; this avoids integral windup
    mMotor.config_IntegralZone(0, 200, kClimberTimeoutMs);
  }

  public void extend() {
    mLogger.verbose("Extending climber to " + HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, HEIGHT_IN_ENCODER_TICKS);
  }

  public void retract() {
    mLogger.verbose("Retracting the climber to " + RETRACTED_HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, RETRACTED_HEIGHT_IN_ENCODER_TICKS);
  }

  public void stop() {
    mLogger.verbose("Stopping the climber");
    mMotor.stopMotor();
  }

  public boolean isExtended() {
    return true;
  }

  public void jog(int direction, double speed) {
    mMotor.set(ControlMode.PercentOutput, speed * direction);
  }

  public void home() {
    mMotor.set(ControlMode.PercentOutput, HOMING_SPEED_PERCENT);
  }

  public void stopHoming() {
    mMotor.stopMotor();
    mMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
