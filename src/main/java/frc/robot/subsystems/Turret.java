package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.OI.OperatorInterface;

/**
 * This Turret singleton extends WPILIB's PID subsystem via @Overriding methods use enable(),
 * disable() and setSetpoint() to control the PID. loop() should be run every tick
 */
public class Turret extends PIDSubsystem {
  private static Turret sInstance;

  private Logger mTurretLogger;
  private WPI_TalonSRX mTurretTalon;
  private Potentiometer mPot;

  // TODO: this
  private Relay cameraLight = new Relay(Constants.cameraRingId);

  // the target position (on a scale from 0 to 100)
  private double mManualTargetPos = 50;

  public static Turret getInstance() {
    if (sInstance == null) {
      sInstance = new Turret();
    }

    return sInstance;
  }

  /** Set up our talon, logger and potentiometer */
  private Turret() {

    // Set PID values
    super(
        new PIDController(
            Config.getInstance().getDouble(Key.OI__VISION__PID__P),
            Config.getInstance().getDouble(Key.OI__VISION__PID__I),
            Config.getInstance().getDouble(Key.OI__VISION__PID__D)));
    mTurretLogger = new Logger("turret");
    mTurretTalon = new WPI_TalonSRX(Config.getInstance().getInt(Key.ROBOT__TURRET__TALON_LOCATION));
    mPot =
        new AnalogPotentiometer(
            Config.getInstance().getInt(Key.ROBOT__POT__LOCATION),
            Config.getInstance().getFloat(Key.ROBOT__POT__RANGE),
            Config.getInstance().getFloat(Key.ROBOT__POT__OFFSET));
  }

  /**
   * Gets the PID value
   *
   * @return the PID value
   */
  @Override
  protected double getMeasurement() {
    // gets the POT value, rounded to 2 decimal places

    // TODO: is this even needed?
    double potValue = Double.parseDouble(String.format("%.2f", this.mPot.get()));
    mTurretLogger.verbose("pot value " + potValue);
    return potValue;
  }

  /** @param output The motor output from the PID to control the motor. */
  @Override
  protected void useOutput(double output, double unused) {
    // limit the output to prevent the motor from going too fast
    output = Math.min(output, Config.getInstance().getDouble(Key.OI__VISION__PID__MAX_SPEED));
    mTurretLogger.verbose("pid out " + output);
    mTurretTalon.set(ControlMode.PercentOutput, output);
  }

  /** @return the raw POT value */
  public double getPotValue() {
    return mPot.get();
  }

  /** Run this every tick. */
  public void loop() {
    // This has to be turned off every tick to keep Mailly's hands from falling off
    OperatorInterface.getInstance().setOperatorRumble(false);

    float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
    float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);

    boolean allowMovement = (mPot.get() < potMax && mPot.get() > potMin);
    mTurretLogger.silly(
        "allow movement "
            + allowMovement
            + " because we got "
            + mPot.get()
            + " inside of "
            + potMin
            + " to "
            + potMax);

    if (allowMovement) {
      if (!this.isEnabled()) enable();
      if (Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)) {
        // vision goes here
      } else {
        setSetpoint(mManualTargetPos);
        if (OperatorInterface.getInstance().getTurretAdjustLeft()) mManualTargetPos -= 2.5;
        if (OperatorInterface.getInstance().getTurretAdjustRight()) mManualTargetPos += 2.5;

        mManualTargetPos = Math.min(Math.max(mManualTargetPos, potMin), potMax);
        mTurretLogger.debug(
            mManualTargetPos
                + " "
                + OperatorInterface.getInstance().getTurretAdjustLeft()
                + " : "
                + OperatorInterface.getInstance().getTurretAdjustRight());
      }
    } else {
      if (this.isEnabled()) disable();
      mTurretLogger.verbose("movement blocked");
      OperatorInterface.getInstance().setOperatorRumble(true);
    }
    // run the PIDSubsystem system's loop
    this.periodic();
  }

  public void setCameraLight(boolean on) {
    if (on) {
      cameraLight.set(Relay.Value.kOn);
    } else {
      cameraLight.set(Relay.Value.kOff);
    }
  }
}
