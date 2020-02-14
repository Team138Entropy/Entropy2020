package frc.robot.subsystems.auto.path;

public class TurnSegment extends Segment {
  private double degrees;
  private boolean done = false;

  public TurnSegment(double degrees) {
    this.degrees = degrees;
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
  }

  @Override
  public void tick() {
    done = true;
  }

  @Override
  public boolean finished() {
    if (done) logger.info("Turn segment finished");
    return done;
  }
}
