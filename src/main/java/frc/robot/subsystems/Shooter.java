package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.SpeedLookupTable;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();

  // Temporary, until default config values are merged
  private static final double P = 3, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.SHOOTER__ROLLER);
  private static final int ROLLER_SLAVE_PORT = Config.getInstance().getInt(Key.SHOOTER__ROLLER_SLAVE);

  // TODO: Tune these values
  private static final int ROLLER_SPEED = 2000; // Encoder ticks per 100ms, change this value
  private static final double TARGET_ROLLER_VELOCITY = 0;

  private static class TurretPosition {
    private double mAzimuth, mElevation;

    public TurretPosition(double azimuth, double elevation) {
      mAzimuth = azimuth;
      mElevation = elevation;
    }

    public double getAzimuth() {
      return mAzimuth;
    }

    public double getElevation() {
      return mElevation;
    }
  }

  @FunctionalInterface
  private interface Turret {
    void set(TurretPosition position);
  }

  @FunctionalInterface
  private interface Vision {
    TurretPosition calcTargetPosition();
  }

  @FunctionalInterface
  private interface Intake {
    void shoveANodeIntoTheThing();
  }

  // TEMPORARY STUFF ENDS HERE

  // Aggregation
  private static Shooter instance;
  private PIDRoller mRoller;
  private TalonSRX mTestRoller;
  private Turret mTurret;
  private Vision mVision;

  private double mRollerVelocity;

  private Shooter() {
    mRoller = new PIDRoller(ROLLER_PORT, ROLLER_SLAVE_PORT, P, I, D);
    mTestRoller = new TalonSRX(ROLLER_PORT);

    // TODO: Replace these with real subsystems
    mTurret =
        position ->
            System.out.println(
                "Setting dummy turret position to ("
                    + position.getAzimuth()
                    + ", "
                    + position.getElevation()
                    + ")");
    mVision =
        () -> {
          System.out.println("Getting dummy vision target");
          return new TurretPosition(0, 0);
        };
  }

  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  /** Tells the turret to move to where the vision system says we should be. */
  public void target() {
    mTurret.set(mVision.calcTargetPosition());
  }

  /** Starts the roller. */
  public void start() {
    System.out.println("Starting roller");
    mRoller.setSpeed(ROLLER_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    System.out.println("Stopping roller");
    mRoller.setSpeed(0);
  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    System.out.println(mRoller.getVelocity());
    // TODO: actually set roller velocity
    return true;// mRollerVelocity > TARGET_ROLLER_VELOCITY;
  }

  // Used in TEST mode only
  public void setOutput(double output) {
    mTestRoller.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
