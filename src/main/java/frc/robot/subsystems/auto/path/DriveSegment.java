package frc.robot.subsystems.auto.path;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

public class DriveSegment extends Segment {
  private double meters; // For cloning

  private double ramp;
  private int targetPosition;
  private int min, max;
  private double P, I, D;
  private double integral = 0, previousError = 0;
  private double previousOutput = 0;

  private boolean done = false;
  private Drive drive;
  private Timer timer;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  public DriveSegment(double meters) {
    this.meters = meters;
    this.drive = Drive.getInstance();
    this.targetPosition = drive.metersToTicks(meters);
    int acceptableError = Config.getInstance().getInt(Config.Key.AUTO__DRIVE_PID_ACCEPTABLE_ERROR);
    this.min = targetPosition - acceptableError;
    this.max = targetPosition + acceptableError;

    this.P = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_P);
    this.I = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_I);
    this.D = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_D);

    this.ramp = Config.getInstance().getDouble(Config.Key.AUTO__DRIVE_PID_RAMP);
    this.timer = new Timer();
  }

  @Override
  public void init() {
    logger.info("Initializing drive segment");
    logger.info("Target: " + targetPosition);

    drive.zeroEncoders();
    try {
      // TODO: Change the calls in zeroEncoders() so that they block until it's done so we don't need this
      Thread.sleep(500);
    } catch(Exception e) {
      e.printStackTrace();
    }

    timer.start();
  }

  @Override
  public void tick() {

    double avgPos = getAveragePosition();

    if (++loggingCount > 5) {
      logger.info(
        "Encoder distances: ("
          + drive.getLeftEncoderDistance()
          + ", "
          + drive.getRightEncoderDistance()
          + ")," + " Average position: " + avgPos);
      loggingCount = 0;
    }

    if (avgPos > min && avgPos < max) {
      logger.verbose(min + " < " + avgPos + " < " + max);
      done = true;
      return;
    }

    /*
    PID stuff.
    We're doing manual PID here because the talon's integrated PID was causing problems.
     */
    double error = targetPosition - avgPos;
    timer.stop();
    this.integral += (error * timer.get());
    double derivative = (error - previousError) / timer.get();
    double inc = P*error + I*integral + D*derivative;
    this.previousError = error;

    double out = previousOutput + inc;
    if ((previousOutput + inc) > ramp) out = previousOutput + ramp;

    if (out > 1) {
      logger.warn("Output was greater than one! Limiting to one so the robot doesn't shit itself");
      out = 1;
    }

    previousOutput = out;

    drive.setOpenLoop(new DriveSignal(out, out));
    timer.reset(); // Does not stop the timer
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Drive segment finished");
      timer.stop();
      timer.reset();
    }

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
