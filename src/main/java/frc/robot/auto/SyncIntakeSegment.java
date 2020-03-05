package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.Storage;

public class SyncIntakeSegment extends Segment {
  private int mBallsToGet;

  public SyncIntakeSegment(int balls){
    mBallsToGet = balls;
  }

  private static boolean active = false;

  public static boolean isActive() {
      return active;
  }

  public static void resetActivatedState() {
      active = false;
  }

  private boolean done = false;

  @Override
  public void init() {
    active = true;
    logger.info("Initializing sync intake segment");

    OperatorInterface.getInstance().overrideIntake(); 
  }

  @Override
  public void tick() {
    if (Storage.getInstance().getBallCount() == mBallsToGet) {
      done = true;
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Intake segment finished");
      OperatorInterface.getInstance().overrideIntake();
      active = false;
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new SyncIntakeSegment(mBallsToGet);
  }
}
