package frc.robot.auto;

import frc.robot.events.Event;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.Storage;

public class AsyncShootSegment extends ShootSegment {

  @Override
  public void init() {
    logger.info("Initializing async shoot segment");

    // OperatorInterface.getInstance().overrideShoot();
    startShooting();
    

    EventWatcherThread.getInstance().registerEvent(new Event() {
      boolean done = false;

      @Override
      public boolean predicate() {
        return Storage.getInstance().isEmpty();
      }

      @Override
      public void run() {
        // OperatorInterface.getInstance().overrideShoot();
        stopShooting();
        resetState(); // This will absolutely cause problems if we have two shoot segments running at the same time
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
