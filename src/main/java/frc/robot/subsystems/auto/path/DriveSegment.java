package frc.robot.subsystems.auto.path;

import frc.robot.Config;
import frc.robot.subsystems.Drive;

public class DriveSegment extends Segment {
  private final int PID_LOOP_ID = 0;
  private double meters; // For cloning
  private int targetPosition;
  private int acceptableError;

  private boolean done;
  private Drive drive;

  public DriveSegment(double meters) {
    this.meters = meters;
    this.drive = Drive.getInstance();
    this.targetPosition = drive.metersToTicks(this.meters);
    this.acceptableError = Config.getInstance().getInt(Config.Key.DRIVE__PID_ACCEPTABLE_ERROR);
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");

    drive.zeroEncoders();
    drive.setTargetPosition(targetPosition);
  }

  @Override
  public void tick() {
    final int min = targetPosition - acceptableError;
    final int max = targetPosition + acceptableError;
    double avgPos = getAveragePosition();

    logger.verbose("Average position: " + avgPos);

    if (avgPos > min && avgPos < max) {
      logger.verbose(min + " < " + avgPos + " < " + max);
      done = true;
    }
  }

  @Override
  public boolean finished() {
    if (done) logger.info("Drive segment finished");
    return done;
  }

  @Override
  public Segment copy() {
    return new DriveSegment(meters);
  }

  /**
   * Returns the average of the left and right encoder distances.
   * @return the average of the encoder distances.
   */
  private double getAveragePosition() {
    return (drive.getLeftEncoderDistance() + drive.getRightEncoderDistance()) / 2;
  }
}
