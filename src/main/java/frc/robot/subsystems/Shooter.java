package frc.robot.subsystems;

import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.SpeedLookupTable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {
  private final SpeedLookupTable mLookupTable = SpeedLookupTable.getInstance();

  // Temporary, until default config values are merged
  private static final double P = 0.5, I = 0, D = 0;

  // TODO: Integrate with other subsystems for real
  // TEMPORARY STUFF BEGINS HERE
  private static final int ROLLER_PORT = Config.getInstance().getInt(Key.SHOOTER__ROLLER_PORT);

  // TODO: Tune these values
  private static final int ROLLER_SPEED = 1; // Encoder ticks per 100ms, change this value
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

  private double mRollerVelocity;

  private boolean mRunning = false;


  private WPI_TalonSRX shooter1;
  private WPI_TalonSRX shooter2;

  private Shooter() {
    //mRoller = new PIDRoller(ROLLER_PORT, P, I, D);
   // mRunning = false;

    shooter1 = new WPI_TalonSRX(5);
    shooter2 = new WPI_TalonSRX(6);
    shooter1.follow(shooter2);

    // TODO: Replace these with real subsystems
    /*
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
      */
  }



  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  /** Tells the turret to move to where the vision system says we should be. */
  public void target() {
    mTurret.set(mVision.calcTargetPosition());
  }

  public synchronized boolean isRunning(){
    return mRunning;
  }

  /** Starts the roller. */
  public synchronized void start() {
   // mRoller.setSpeed(ROLLER_SPEED);
   //shooter1.set
   shooter1.set(ControlMode.PercentOutput, ROLLER_SPEED);

  }

  /** Stops the roller. */
  public synchronized void stop() {
    //mRoller.setSpeed(0);
    shooter1.set(ControlMode.PercentOutput, 0);

  }

  /** Returns whether roller is at full speed. */
  public boolean isAtVelocity() {
    return mRollerVelocity > TARGET_ROLLER_VELOCITY;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}
