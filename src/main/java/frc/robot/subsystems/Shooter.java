package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {

  // Temporary, until default config values are merged
  private static final double P = 0.5, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = 0;
  private static final int MAX_CAPACITY = 5;

  // TODO: Tune these values
  private static final int ROLLER_SPEED = 1; // Encoder ticks per 100ms, change this value
  private static final double SPINUP_DELAY_SECONDS = 0.5;
  private static final double FIRE_DURATION_SECONDS = 0.5;

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
  private int mBuffer = 0;
  private Timer mSpinUpTimer;
  private Timer mFireTimer;

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

  /**
   * Call this in the robot loop.
   */
  public void periodic() {

    // Check if we're done spinning up yet
    if (mState == State.SPINNING_UP && mSpinUpTimer.get() >= SPINUP_DELAY_SECONDS) {
      mState = State.FULL_SPEED;
      mSpinUpTimer.stop();
      mSpinUpTimer.reset();
    }

    // Check if we're done firing yet
    if (mState == State.FIRING && mFireTimer.get() >= FIRE_DURATION_SECONDS) {
      mState = State.IDLE;
      mFireTimer.stop();
      mFireTimer.reset();
    }

    // Handle buffered fire operations
    if (mBuffer > 0) {

      // If we haven't started spinning up yet
      if (mState != State.SPINNING_UP) {
        start();
        mSpinUpTimer.reset();
        mSpinUpTimer.start();
      }

      if (mState == State.FULL_SPEED) {
        mState = State.FIRING;
        mIntake.shoveANodeIntoTheThing();
        mFireTimer.start();
        mBuffer--;
      }
    } else {
      // Handle the case where the buffer was reset while we were doing something
      if (mState == State.SPINNING_UP || mState == State.FULL_SPEED) {
        stop();
        mSpinUpTimer.stop();
        mSpinUpTimer.reset();
      }
    }
  }

  /** Buffers another fire operation. */
  public void fireSingle() {
    mBuffer++;
  }

  /**
   * Equivalent to calling {@link #resetBuffer()} and then calling {@link #fireSingle()} a number of
   * times equal to the number of balls in the storage mechanism.
   */
  public void fireAuto() {
    resetBuffer();
    for (int i = 0; i < MAX_CAPACITY; i++) {
      fireSingle();
    }
  }

  /**
   * Resets the firing buffer. Has the effect of cancelling any buffered fire operations, including
   * automatic fire.
   */
  public void resetBuffer() {
    mBuffer = 0;
  }

  /** The same as {@link #resetBuffer()}. Exists to make calling code more declarative. */
  public void stopFiring() {
    resetBuffer();
  }

  /** Tells the turret to move to where the vision system says we should be. */
  public void target() {
    mTurret.set(mVision.calcTargetPosition());
  }

  /** Starts the roller. */
  private void start() {
    mRoller.setSpeed(ROLLER_SPEED);
  }

  /** Stops the roller. */
  private void stop() {
    mRoller.setSpeed(0);
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}
