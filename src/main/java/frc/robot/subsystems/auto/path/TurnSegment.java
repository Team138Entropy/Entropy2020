package frc.robot.subsystems.auto.path;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

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

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  public TurnSegment(double degrees) {
    this.degrees = degrees % 360;
    this.drive = Drive.getInstance();
    this.gyro = Robot.getGyro();
    double acceptableError =
        Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_ACCEPTABLE_ERROR);
    this.minAcceptable = this.degrees - acceptableError;
    this.maxAcceptable = this.degrees + acceptableError;

    this.P = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_P);
    this.I = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_I);
    this.D = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_D);

    this.maxOutput = Config.getInstance().getDouble(Config.Key.AUTO__TURN_PID_MAX);
    this.timer = new Timer();
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
    logger.info("Target: " + degrees + "deg");

    gyro.reset();
    timer.start();
  }

  @SuppressWarnings("DuplicatedCode") // Marks some code as duplicate of code in DriveSegment.tick()
  @Override
  public void tick() {
    double heading = gyro.getAngle() % 360;

    if (++loggingCount > 5) {
      logger.info("Relative heading: " + heading + "deg");
      loggingCount = 0;
    }

    if (heading > minAcceptable && heading < maxAcceptable) {
      logger.verbose(minAcceptable + " < " + heading + " < " + maxAcceptable);
      done = true;
      return;
    }

    double error = degrees - heading;
    timer.stop();
    this.integral += (error * timer.get());
    double derivative = (error - previousError) / timer.get();
    double out = (P * error) + (I * integral) + (D * derivative);

    if (out > maxOutput) out = maxOutput;

    if (out > 1) {
      logger.warn("Output was greater than one! Limiting to one so the robot doesn't shit itself");
      logger.warn("out: " + out);
      out = 1;
    }

    drive.setOpenLoop(new DriveSignal(-out, out));
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
