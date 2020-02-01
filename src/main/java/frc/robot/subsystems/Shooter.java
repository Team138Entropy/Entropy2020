package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {

  // Temporary, until default config values are merged
  private static final double P = 0.5, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = 69;
  private static final int MAX_CAPACITY = 5;

  // TODO: Tune these values
  private static final int ROLLER_SPEED = 1; // Encoder ticks per 100ms, change this value
  private static final double SPINUP_DELAY_SECONDS = 0.5;
  private static final double FIRE_DURATION_SECONDS = 0.5;
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
  private Turret mTurret;
  private Vision mVision;
  private Intake mIntake;

  // State variables
  public enum State {
    IDLE,
    FULL_SPEED,
    SPINNING_UP,
    FIRING
  }

  private State mState = State.IDLE;
  private Timer mSpinUpTimer;
  private Timer mFireTimer;
  private double mRollerVelocity;

  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  public State getState() {
    return mState;
  }

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
    mIntake = () -> System.out.println("Shoving a ball into the thing");

    mSpinUpTimer = new Timer();
    mFireTimer = new Timer();
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

  /** Starts the firing timer and roller.  */
  public void shootBall() {
    mFireTimer.start();
    start();
  }

  /** Returns whether the firing timer has run longer than the duration. */
  public boolean isDoneShooting() {
    if (mFireTimer.get() >= FIRE_DURATION_SECONDS * 1000) {
      stop();
      mFireTimer.stop();
      mFireTimer.reset();
    }
    return mFireTimer.get() >= FIRE_DURATION_SECONDS * 1000;
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}
