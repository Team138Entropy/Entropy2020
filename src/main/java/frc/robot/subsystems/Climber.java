package frc.robot.subsystems;

public class Climber extends Subsystem {

  /** Aggregation */
  private static Climber sInstance;

  public Climber() {
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void extend() {
  }

  public void retract() {
  }

  public void stop() {
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
