package frc.robot.subsystems.auto.path;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class TurnSegment extends Segment {
  private double degrees;

  private double maxOutput;
  private double minAcceptable, maxAcceptable;
  private double P, I, D;
  private double integral = 0, previousError = 0;

  private boolean done = false;
  private Drive drive;
  private Gyro gyro;
  private Timer timer;


  public TurnSegment(double degrees) {
    this.degrees = degrees;
    this.drive = Drive.getInstance();
    this.gyro = Robot.getGyro();
    double acceptableError = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_ACCEPTABLE_ERROR);
    this.minAcceptable = degrees - acceptableError;
    this.maxAcceptable = degrees + acceptableError;

    this.P = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_P);
    this.I = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_I);
    this.D = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_D);

    this.maxOutput = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_MAX);
    this.timer = new Timer();
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
    logger.info("Target: " + degrees + "deg (clockwise)");

    gyro.reset();
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

  @Override
  public Segment copy() {
    return new TurnSegment(degrees);
  }
}
