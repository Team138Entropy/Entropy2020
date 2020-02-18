package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.SpeedLookupTable;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();

  // Temporary, until default config values are merged
  private static final double MAX_SPEED = 2445d;
  private static final double SPEED_DEADBAND = 30;
  private static final int SPEED_DEADBAND_DELAY = 10;
  private static final double FEEDFORWARD = 1023d / MAX_SPEED, P = (.5 * 1023) / 50, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.SHOOTER__ROLLER);
  private static final int ROLLER_SLAVE_PORT =
      Config.getInstance().getInt(Key.SHOOTER__ROLLER_SLAVE);

  // TODO: Tune these values
  private static final int DEFAULT_ROLLER_SPEED = 2000; // Encoder ticks per 100ms, change this value
  private int mVelocityAdjustment = 0;
  private static final int VELOCITY_ADJUSTMENT_BUMP = Config.getInstance().getInt(Key.SHOOTER__VELOCITY_ADJUSTMENT);

  private boolean mHasHadCurrentDrop = false;

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
  private int mTimeSinceWeWereAtVelocity = SPEED_DEADBAND_DELAY;
  private void x(){}
  private Shooter() {
    mRoller = new PIDRoller(ROLLER_PORT, ROLLER_SLAVE_PORT, P, I, D, FEEDFORWARD);
    mTestRoller = new TalonSRX(ROLLER_PORT);

    // TODO: Replace these with real subsystems
    mTurret =
        position -> 
            x();
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
    mRoller.setSpeed(getAdjustedVelocitySetpoint());
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.setSpeed(0);
  }

  public int getSpeed() {
    return mRoller.getVelocity();
  }

  private int getAdjustedVelocitySetpoint() {
    int speed = (int) Math.round(SpeedLookupTable.getInstance().getSpeedFromDistance(mVision.calcTargetPosition().getDistance()));
    return speed + mVelocityAdjustment;
  }

  public void increaseVelocity() {
    mVelocityAdjustment += VELOCITY_ADJUSTMENT_BUMP;
  }

  public void decreaseVelocity() {
    mVelocityAdjustment -= VELOCITY_ADJUSTMENT_BUMP;
  }

  public void resetVelocity() {
    mVelocityAdjustment = 0;
  }

  public int getVelocityAdjustment() {
    return mVelocityAdjustment;
  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    // determine if we're at the target velocity by looking at the difference between the actual and
    // expected
    // and if that difference is less than SPEED_DEADBAND, we are at the velocity
    boolean isAtVelocity = Math.abs(mRoller.getVelocity() - getAdjustedVelocitySetpoint()) < SPEED_DEADBAND;

    // here's the problem:
    // the velocity we get often bounces around, causing breif moments when we think we aren't there
    // add a "delay" where we still consider ourselves to be at the velocity if we were there in the
    // last SPEED_DEADBAND_DELAY ticks

    SmartDashboard.putNumber("Velocity Countdown", mTimeSinceWeWereAtVelocity);
    SmartDashboard.putBoolean("Has Had Current Drop", mHasHadCurrentDrop);

    if (isAtVelocity) {
      // decrement
      mTimeSinceWeWereAtVelocity--;
    } else {
      // reset the time since we were at velocity
      mTimeSinceWeWereAtVelocity = SPEED_DEADBAND_DELAY;
    }
    // if the time is at least 0, we are "at velocity"
    boolean isAtVelocityDebounced = mTimeSinceWeWereAtVelocity <= 0;

    return isAtVelocityDebounced;
  }

  public boolean isBallFired(){
    return Math.abs(mRoller.getVelocity() - getAdjustedVelocitySetpoint()) >= (SPEED_DEADBAND + 50);
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