package frc.robot.auto;

import frc.robot.events.Event;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.Storage;

public class AsyncShootSegment extends ShootSegment {

  @Override
  public void init() {
    logger.info("Initializing async shoot segment");

    startShooting();

    EventWatcherThread.getInstance()
        .registerEvent(
            new Event() {

              @Override
              public boolean predicate() {
                return Storage.getInstance().isEmpty();
              }

              @Override
              public void run() {
                logger.info("Async shoot segment finished");
                stopShooting();
                resetState(); // This will absolutely cause problems if we have two shoot segments
                // running at the same time
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
    return new AsyncShootSegment();
  }
}
