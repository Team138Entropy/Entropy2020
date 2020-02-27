package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.events.Event;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.Storage;

public class AsyncShootSegment extends Segment {

  @Override
  public void init() {
    logger.info("Initializing async shoot segment");

    OperatorInterface.getInstance().overrideShoot();

    EventWatcherThread.getInstance().registerEvent(new Event() {
      boolean done = false;

      @Override
      public boolean predicate() {
        return Storage.getInstance().isEmpty();
      }

      @Override
      public void run() {
        OperatorInterface.getInstance().overrideShoot();
        done = true;
      }

      @Override
      public boolean pruneMe() {
        if (done) {
          logger.info("Async shoot segment finished");
        }

        return done;
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
    return new AsyncShootSegment();
  }
}
