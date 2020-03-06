package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

/** A type of {@link Segment} for driving straight. */
public class DriveSegment extends Segment {
  private double feet; // For cloning
  private int cruise;
  private int accel;
  private boolean test;

  private int targetPosition;
  private int min, max;

  private boolean done = false;
  private Drive drive;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  @SuppressWarnings("FieldCanBeLocal")
  private final int LOGGING_COUNT_MODULUS = 5;

  // The number of ticks for which we've been within the acceptable range
  private int debounceCount = 0;

  public DriveSegment(double feet, int cruise, int accel) {
    this.feet = feet;
    this.drive = Drive.getInstance();
    this.targetPosition = drive.feetToTicks(feet);
    int acceptableError = 50;
    this.min = targetPosition - acceptableError;
    this.max = targetPosition + acceptableError;

    this.cruise = cruise;
    this.accel = accel;
    this.test = true;
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");
    logger.info("Target: " + targetPosition);

    drive.zeroEncoders();
    drive.setCruiseAndAcceleration(cruise, accel);

    logger.info("Cruise: " + cruise);
    logger.info("Accel: " + accel);

    drive.setMotionMagicTarget(targetPosition, targetPosition);
  }

  @Override
  public void tick() {

    int left = drive.getLeftEncoderDistance();
    int right = -drive.getRightEncoderDistance();

    if (++loggingCount > LOGGING_COUNT_MODULUS) {
      logger.info(
          "Encoder distances: ("
              + drive.getLeftEncoderDistance()
              + ", "
              + -drive.getRightEncoderDistance()
              + ")");
      loggingCount = 0;
    }

    // SmartDashboard.putNumber("left encoder", left);
    // SmartDashboard.putNumber("right encoder", right);
    SmartDashboard.putNumber("Average encoder distance", (left + right) / 2.0);

    if (acceptable(left) && acceptable(right)) {
      logger.verbose("Positions in acceptable range for " + ++debounceCount + " tick(s)");

      if (debounceCount >= Constants.AUTO_DEBOUNCE_TICKS) {
        done = true;
        drive.setOpenLoop(DriveSignal.BRAKE);
      }
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Drive segment finished");
      drive.resetCruiseAndAccel();
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new DriveSegment(feet, cruise, accel);
  }

  private boolean acceptable(int ticks) {
    return (ticks > min && ticks < max);
  }
}
