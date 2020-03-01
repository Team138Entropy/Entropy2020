package frc.robot.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

/** A type of {@link Segment} for turning in place. */
public class TurnSegment extends Segment {
  private static final double ACCEPTABLE_ERROR = 2;

  private double degrees; // For cloning

  private int targetPosition;
  private double minAcceptable, maxAcceptable;

  private boolean done = false;
  private Drive drive;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  // The number of ticks for which we've been within the acceptable range
  private int debounceCount = 0;

  public TurnSegment(double degrees) {
    // this.trackWidth = Config.getInstance().getDouble(Config.Key.ROBOT__REAL_TRACK_WIDTH);
    // this.ticksPerMeter = Config.getInstance().getDouble(Config.Key.DRIVE__TICKS_PER_METER);

    this.degrees = degrees % 360;
    this.drive = Drive.getInstance();
    targetPosition = degreesToTicks(degrees);

    // double acceptableError =
    //    Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_ACCEPTABLE_ERROR);
    this.minAcceptable = this.degrees - ACCEPTABLE_ERROR;
    this.maxAcceptable = this.degrees + ACCEPTABLE_ERROR;
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
    logger.info("Target angle: " + degrees + "deg");
    logger.info("Target distance: " + targetPosition);

    drive.configP(10);
    drive.configI(0);

    drive.zeroEncoders();
    drive.setMotionMagicTarget(-targetPosition, targetPosition); // Counterclockwise
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

    double angle = calcAngle(-left, right);

    if (angle > minAcceptable && angle < maxAcceptable) {
      logger.verbose("Angle in acceptable range for " + ++debounceCount + " tick(s)");

      if (debounceCount >= Constants.AUTO_DEBOUNCE_TICKS) {
        done = true;
        drive.setOpenLoop(DriveSignal.BRAKE);
      }
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Turn segment finished");
      drive.resetPID();
    }
    return done;
  }

  @Override
  public Segment copy() {
    return new TurnSegment(degrees);
  }

  private int degreesToTicks(double theta) {
    double r = drive.feetToTicks(Constants.REAL_TRACK_WIDTH) / 2.0;
    return (int) Math.round((Math.PI * r * theta) / 180.0); // Magic!
  }

  private static class Point {
    public double x, y;

    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  // Heavy: Are you sure this will work?
  // Medic: Ha ha, I have no idea!
  private double calcAngle(double leftArc, double rightArc) {
    double r = drive.feetToTicks(Constants.REAL_TRACK_WIDTH) / 2.0;
    
    double leftTheta = (degreesToTicks(180) + leftArc) / r;
    double rightTheta = rightArc / r;

    Point leftPoint = new Point(
      Math.cos(leftTheta),
      Math.sin(leftTheta)
    );

    Point rightPoint = new Point(
      Math.cos(rightTheta),
      Math.sin(rightTheta)
    );

    double slope = (rightPoint.y - leftPoint.y) / (rightPoint.x - leftPoint.x);

    return Math.toDegrees(Math.atan(slope));
  }
}
