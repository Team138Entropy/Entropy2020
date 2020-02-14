package frc.robot.subsystems.auto.path;

import frc.robot.Logger;
import frc.robot.subsystems.Drive;

import javax.annotation.Nonnull;

public class DriveSegment extends Segment {
  private double meters;
  private boolean done;
  private Drive drive;

  public DriveSegment(double meters) {
    this.meters = meters;
    this.drive = Drive.getInstance();
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");
  }

  @Override
  public void tick() {
    done = true;
  }

  @Override
  public boolean finished() {
    if (done) logger.info("Drive segment finished");
    return done;
  }

  private double getDistance;
}
