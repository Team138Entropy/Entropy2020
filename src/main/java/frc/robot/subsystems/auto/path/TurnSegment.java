package frc.robot.subsystems.auto.path;

import frc.robot.Config;
import frc.robot.subsystems.Drive;

public class TurnSegment extends Segment {
  private final double trackWidth = 0.58;
  private final int ticksPerMeter = 640;
  private final double acceptableError = 5; 


  private double degrees; // For cloning

  private int targetPosition;
  private double minAcceptable, maxAcceptable;

  private boolean done = false;
  private Drive drive;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  public TurnSegment(double degrees) {
    //this.trackWidth = Config.getInstance().getDouble(Config.Key.ROBOT__REAL_TRACK_WIDTH);
    //this.ticksPerMeter = Config.getInstance().getDouble(Config.Key.DRIVE__TICKS_PER_METER);

    this.degrees = degrees % 360;
    targetPosition = degreesToTicks(degrees);
    this.drive = Drive.getInstance();
    //double acceptableError =
    //    Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_ACCEPTABLE_ERROR);
    this.minAcceptable = this.degrees - acceptableError;
    this.maxAcceptable = this.degrees + acceptableError;
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
    logger.info("Target: " + degrees + "deg");
  }

  @Override
  public void tick() {
    done = true; // So that it doesn't halt at this segment
  }

  @Override
  public boolean finished() {
    if (done) logger.info("Turn segment finished");
    return done;
  }

  @Override
  public Segment copy() {
    return new TurnSegment(degrees);
  }

  private int degreesToTicks(double theta) {
    double r = metersToTicks(trackWidth) / 2.0;
    return (int) Math.round((Math.PI * r * theta) / 180.0); // Magic!
  }

  private int metersToTicks(double meters) {
    return (int) Math.round(ticksPerMeter * meters);
  }
}
