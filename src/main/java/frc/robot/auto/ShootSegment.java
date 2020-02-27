package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.Storage;

public class ShootSegment extends Segment {
  private OperatorInterface operatorInterface = OperatorInterface.getInstance();

  private boolean done = false;

  @Override
  public void init() {
    logger.info("Initializing shoot segment");

    operatorInterface.overrideShoot(); // Simulate first button press
  }

  @Override
  public void tick() {
    if (Storage.getInstance().isEmpty()) {
      operatorInterface.overrideShoot(); // Simulate last button press
      done = true;
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Shoot segemnt finished");
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new ShootSegment();
  }
}
