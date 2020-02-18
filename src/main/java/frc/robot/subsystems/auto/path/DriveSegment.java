package frc.robot.subsystems.auto.path;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

public class DriveSegment extends Segment {
  private double meters; // For cloning

  private int targetPosition;
  private int min, max;

  private boolean done = false;
  private Drive drive;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  public DriveSegment(double meters) {
    this.meters = meters;
    this.drive = Drive.getInstance();
    this.targetPosition = drive.metersToTicks(meters);
    int acceptableError = Config.getInstance().getInt(Config.Key.AUTO__DRIVE_PID_ACCEPTABLE_ERROR);
    this.min = targetPosition - acceptableError;
    this.max = targetPosition + acceptableError;
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");
    logger.info("Target: " + targetPosition);

    drive.zeroEncoders();
    try {
      // TODO: Change the calls in zeroEncoders() so that they block until it's done so we don't
      // need this
      Thread.sleep(500);
    } catch (Exception e) {
      e.printStackTrace();
    }

  }

  @SuppressWarnings("DuplicatedCode") // Marks some code as duplicate of code in TurnSegment.tick()
  @Override
  public void tick() {

    double avgPos = getAveragePosition();

    if (++loggingCount > 5) {
      logger.info(
          "Encoder distances: ("
              + drive.getLeftEncoderDistance()
              + ", "
              + drive.getRightEncoderDistance()
              + "),"
              + " Average position: "
              + avgPos);
      loggingCount = 0;
    }

    if (avgPos > min && avgPos < max) {
      logger.verbose(min + " < " + avgPos + " < " + max);
      done = true;
      drive.setOpenLoop(DriveSignal.BRAKE);
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Drive segment finished");
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new DriveSegment(meters);
  }

  /**
   * Returns the average of the left and right encoder distances.
   *
   * @return the average of the encoder distances.
   */
  private double getAveragePosition() {
    return (drive.getLeftEncoderDistance() + drive.getRightEncoderDistance()) / 2;
  }
}
