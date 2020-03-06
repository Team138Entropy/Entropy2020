package frc.robot.auto;

public class SyncIntakeSegment extends Segment {
  public SyncIntakeSegment() {}

  private static boolean active = false;

  public static boolean isActive() {
    return active;
  }

  public static void resetActivatedState() {
    active = false;
  }

  private boolean done = false;

  @Override
  public void init() {
    active = true;
    logger.info("Initializing sync intake segment");
  }

  @Override
  public void tick() {
    done = !IntakeSegment.isActive();
  }

  @Override
  public boolean finished() {
    if (done) {
      active = false;
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new SyncIntakeSegment();
  }
}
