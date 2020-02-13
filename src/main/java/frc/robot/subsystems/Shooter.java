package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.SpeedLookupTable;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();

  // Temporary, until default config values are merged
  private final double P = 0.5, I = 0, D = 0;



  // TODO: Tune these values
  private final int ROLLER_SPEED = 1024; // Encoder ticks per 100ms, change this value
  private final double TARGET_ROLLER_VELOCITY = 0;

  

  // Aggregation
  private static Shooter instance;
  private PIDRoller mRoller;
  private double mRollerVelocity;

  private Shooter() {
    mRoller = new PIDRoller(Constants.kShooterMasterTalon, Constants.kShooterSlaveTalon, P, I, D);
  }

  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }


  /** Starts the roller. */
  public void start() {
    System.out.println("Starting roller");
    mRoller.setSpeed(ROLLER_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    System.out.println("Stopping roller");
    mRoller.Stop();
  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    // TODO: actually set roller velocity
    return true;// mRollerVelocity > TARGET_ROLLER_VELOCITY;
  }

  // Used in TEST mode only
  public void setOutput(double output) {
   // mTestRoller.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}