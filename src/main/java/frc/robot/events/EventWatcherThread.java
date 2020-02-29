package frc.robot.events;

import java.util.ArrayList;

public class EventWatcherThread extends Thread {
  // IMPORTANT: For IntakeSegment to work reliably, the delay between its event calls *must* be less than the robot loop period (~20ms)!
  // That means this value should be set considerably lower to account for delays caused by other events.
  // FIXME: Do this the right way so we don't have to restrict the timing
  private static final int LOOP_PERIOD_MS = 10;

  private static EventWatcherThread instance;

  private ArrayList<Event> queue;

  public static synchronized EventWatcherThread getInstance() {
    if (instance == null) {
      instance = new EventWatcherThread();
      instance.start();
    }

    return instance;
  }

  private EventWatcherThread() {
    queue = new ArrayList<>();
  }

  @Override
  public void run() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        for (Event e : queue) {
          if (e.predicate()) {
            e.run();
          
            if (e.pruneMe()) {
              queue.remove(e);
            }
          }
        }

        wait(LOOP_PERIOD_MS);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }
  }

  public synchronized void registerEvent(Event e) {
    queue.add(e);
  }

  public synchronized void unRegisterEvent(Event e) {
    queue.remove(e);
  }
}
