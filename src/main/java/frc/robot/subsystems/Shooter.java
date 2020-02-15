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
  private static final double MAX_SPEED = 2445d;
  private static final double SPEED_DEADBAND = 30;
  private static final int SPEED_DEADBAND_DELAY = 5;
  private static final double FEEDFORWARD = 1023d / MAX_SPEED, P = (.5 * 1023) / 50, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.SHOOTER__ROLLER);
  private static final int ROLLER_SLAVE_PORT =
      Config.getInstance().getInt(Key.SHOOTER__ROLLER_SLAVE);

  // TODO: Tune these values
  private static final int ROLLER_SPEED = 2000; // Encoder ticks per 100ms, change this value

  private static class TurretPosition {
    private double mAzimuth, mDistance;

    public TurretPosition(double azimuth, double distance) {
      mAzimuth = azimuth;
      mDistance = distance;
    }

    public double getAzimuth() {
      return mAzimuth;
    }

    public double getDistance() {
      return mDistance;
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
  private int mTimeSinceWeWereAtVelocity = 0;

  private Shooter() {
    mRoller = new PIDRoller(ROLLER_PORT, ROLLER_SLAVE_PORT, P, I, D, FEEDFORWARD);
    mTestRoller = new TalonSRX(ROLLER_PORT);

    // TODO: Replace these with real subsystems
    mTurret =
        position ->
            System.out.println(
                "Setting dummy turret position to ("
                    + position.getAzimuth()
                    + ", "
                    + position.getDistance()
                    + ")");
    mVision =
        () -> {
          // System.out.println("Getting dummy vision target");
          return new TurretPosition(0, MAX_SPEED);
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
    double speed = SpeedLookupTable.getInstance().getSpeedFromDistance(mVision.calcTargetPosition().getDistance());
    mRoller.setSpeed((int) Math.round(speed));
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.setSpeed(0);
  }

  public int getSpeed() {
    return mRoller.getVelocity();
  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    // determine if we're at the target velocity by looking at the difference between the actual and
    // expected
    // and if that difference is less than SPEED_DEADBAND, we are at the velocity
    boolean isAtVelocity = Math.abs(mRoller.getVelocity() - ROLLER_SPEED) < SPEED_DEADBAND;

    // here's the problem:
    // the velocity we get often bounces around, causing breif moments when we think we aren't there
    // add a "delay" where we still consider ourselves to be at the velocity if we were there in the
    // last SPEED_DEADBAND_DELAY ticks

    if (isAtVelocity) {
      // reset the time since we were at velocity
      mTimeSinceWeWereAtVelocity = SPEED_DEADBAND_DELAY;
    } else {
      // decrement
      mTimeSinceWeWereAtVelocity--;
    }
    // if the time is at least 0, we are "at velocity"
    return mTimeSinceWeWereAtVelocity > 0;
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
