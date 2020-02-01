package frc.robot.subsystems;


import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.SpeedLookupTable;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();


  // Aggregation
  private static Shooter instance;
  private PIDRoller mRoller;
  private Turret mTurret;
  private Vision mVision;

  private double mRollerVelocity;

  private Shooter() {
    mRoller = new PIDRoller(ROLLER_PORT, P, I, D);

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
    mRoller.setSpeed(ROLLER_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.setSpeed(0);
  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    return mRollerVelocity > TARGET_ROLLER_VELOCITY;
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}