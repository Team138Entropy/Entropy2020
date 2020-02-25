package frc.robot.subsystems.auto.path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

public class DriveSegment extends Segment {
  private final int acceptableError = 50;

  private double feet; // For cloning

  private int targetPosition;
  private int min, max;

  private boolean done = false;
  private Drive drive;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  // The number of ticks for which we've been within the acceptable range
  private int debounceCount = 0;

  public DriveSegment(double feet) {
    this.feet = feet;
    this.drive = Drive.getInstance();
    this.targetPosition = drive.feetToTicks(feet);
    this.min = targetPosition - acceptableError;
    this.max = targetPosition + acceptableError;
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");
    logger.info("Target: " + targetPosition);

    drive.zeroEncoders();
    drive.setTargetPosition(targetPosition);
  }

  @Override
  public void tick() {

    int left = drive.getLeftEncoderDistance();
    int right = drive.getRightEncoderDistance();

    if (++loggingCount > 5) {
      logger.info(
          "Encoder distances: ("
              + drive.getLeftEncoderDistance()
              + ", "
              + drive.getRightEncoderDistance()
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
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new DriveSegment(feet);
  }



  private boolean acceptable(int ticks) {
    return (ticks > min && ticks < max);
  }
}
