package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Logger;
import edu.wpi.first.wpilibj.Encoder;



/**
 * This Turret singleton extends WPILIB's PID subsystem via @Overriding methods use enable(),
 * disable() and setSetpoint() to control the PID. loop() should be run every tick
 */
public class Turret extends PIDSubsystem {
  private static Turret sInstance;

  private final Logger mTurretLogger;
  private final WPI_TalonSRX mTurretTalon;
  private final Encoder mTurretEncoder;
  private Potentiometer mPot;



 // private Relay cameraLight = new Relay(Constants.kCameraRingId);

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
          Constants.kPIDController_P,
          Constants.kPIDController_I,
          Constants.kPIDController_D
        ));
    mTurretLogger = new Logger("turret");
    mTurretTalon = new WPI_TalonSRX(Constants.kTurretTalonMotorPort);
    mTurretEncoder = new Encoder(Constants.kTurretEncoderA, Constants.kTurrentEncoderB, false);

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
    //output = Math.min(output, Config.getInstance().getDouble(Key.OI__VISION__PID__MAX_SPEED));
    //mTurretLogger.verbose("pid out " + output);
    //mTurretTalon.set(ControlMode.PercentOutput, output);
  }

  /** @return the raw POT value */
  public double getPotValue() {
    return mPot.get();
  }

  /** Run this every tick. */
  public void loop() {
    //float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
    //float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);
    /*
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
    }
    // run the PIDSubsystem system's loop
    this.periodic();
      */
  }

  public void setCameraLight(boolean on) {
    //cameraLight.set(on ? Relay.Value.kOn : Relay.Value.kOff);
  }
}
