package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.SpeedLookupTable;
import frc.robot.Constants;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();

  private final double MAX_SPEED = 3700.0;
  // private static final double SPEED_DEADBAND = 20;
  private final double SPEED_DEADBAND = 5;
  private final double DROP_DEADBAND = 250;
  private final int SPEED_DEADBAND_DELAY = 5;
  private final double FEEDFORWARD = 1023d / MAX_SPEED;
  // private static final double P = (.3 * 1023) / 50;
  // private static final double I = 0.2;
  // private static final double D = 0.1;
  private final double P = 0;
  private final double I = 0;
  private final double D = 0;

  // a minimum acountdown
  private static final int MIN_SHOT_COUNTDOWN = 100;
  private int mShotCountdown = MIN_SHOT_COUNTDOWN;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private final int ROLLER_PORT = Constants.Talon_Shooter_Master;
  private final int ROLLER_SLAVE_PORT = Constants.Talon_Shooter_Slave;

  // TODO: Tune these values
  private final int DEFAULT_ROLLER_SPEED =
      2000; // Encoder ticks per 100ms, change this value
  private int mVelocityAdjustment = 0;
  private final int VELOCITY_ADJUSTMENT_BUMP =
      Config.getInstance().getInt(Key.SHOOTER__VELOCITY_ADJUSTMENT);

  private boolean mHasHadCurrentDrop = false;

  // Aggregation
  private static Shooter instance;
  private final PIDRoller mRoller;
  private double mDistance = 0;
  private int mTimeSinceWeWereAtVelocity = SPEED_DEADBAND_DELAY;


  private Shooter() {
    mRoller = new PIDRoller(ROLLER_PORT, ROLLER_SLAVE_PORT, P, I, D, FEEDFORWARD);

  }

  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }


  /** Starts the roller. */
  public void start() {
    mRoller.setSpeed(getAdjustedVelocitySetpoint());
    // mRoller.setPercentOutput(1);


    SmartDashboard.putNumber("ShooterCurrent", mRoller.getCurrent());
  }

  /** Stops the roller. */
  public void stop() {
    mRoller.setSpeed(0);
  }

  public void updateDistance(double dist){
    mDistance = dist;
  }

  public int getSpeed() {
    return mRoller.getVelocity();
  }

  private int getAdjustedVelocitySetpoint() {
    double distance = mDistance; //for now
    
    int speed =
        (int)
            Math.round(
                SpeedLookupTable.getInstance()
                    .getSpeedFromDistance(distance));
    
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
  //UPDATE: 
  public boolean isAtVelocity() {

    //New Concept: Velocity FLOOR
    //our velocity setpoint will be slightly higher than it needs to be
    // allow velocity to be sliughtly lower, but operate as a floor
    boolean isAtVelocity = (mRoller.getVelocity() - (getAdjustedVelocitySetpoint()/* - 15*/) >= 0);
    // return isAtVelocity;
    
    SmartDashboard.putNumber("Shot Countdown", mShotCountdown);
    // determine if we're at the target velocity by looking at the difference between the actual and
    // expected
    // and if that difference is less than SPEED_DEADBAND, we are at the velocity

    // boolean isAtVelocity =
        // Math.abs(mRoller.getVelocity() - getAdjustedVelocitySetpoint()) < SPEED_DEADBAND;

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

    //return false;
  }

  public boolean isBallFired(){
    boolean didDropVelocity = Math.abs(mRoller.getVelocity() - getAdjustedVelocitySetpoint()) >= (DROP_DEADBAND);
    boolean ballFired = didDropVelocity;
    return ballFired;
  }

  // Used in TEST mode only
  public void setOutput(double output) {
    mRoller.setPercentOutput(output);
    mRoller.getVelocity();
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
