package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Config;
import frc.robot.Config.Key;

/** Add your docs here. */
public class Intake extends Subsystem {

  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.INTAKE__ROLLER_PORT);

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

  public void start() {
    mRoller.set(ControlMode.PercentOutput, ROLLER_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}
