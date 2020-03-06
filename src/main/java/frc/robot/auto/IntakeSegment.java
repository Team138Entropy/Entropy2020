package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.events.Event;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.Storage;

public class IntakeSegment extends Segment {

  private int ballsToGet;

  private static boolean active = false;

  public static boolean isActive() {
    return active;
  }

  public static void resetActivatedState() {
    active = false;
  }

  public IntakeSegment(int balls) {
    this.ballsToGet = balls;
  }

  @Override
  public void init() {
    logger.info("Initializing intake segment (" + ballsToGet + " balls)");
    active = true;

    OperatorInterface.getInstance().overrideIntake(); // First button press

    EventWatcherThread.getInstance()
        .registerEvent(
            new Event() {

              private int debugCount = 0;

              @Override
              public boolean predicate() {
                return Storage.getInstance().getBallCount() == ballsToGet;
              }

              @Override
              public void run() {
                logger.info("Intake segment finished");
                OperatorInterface.getInstance().overrideIntake(); // Second button press
                active = false;
              }

              @Override
              public boolean pruneMe() {
                return true;
              }
            });
  }

  @Override
  public void tick() {}

  @Override
  public boolean finished() {
    return true;
  }

  @Override
  public Segment copy() {
    return new IntakeSegment(ballsToGet);
  }
}
