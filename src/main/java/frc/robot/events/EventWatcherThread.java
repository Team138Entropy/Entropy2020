package frc.robot.events;

import java.util.ArrayList;

public class EventWatcherThread extends Thread {
  // IMPORTANT: For IntakeSegment to work reliably, the delay between its event calls *must* be less
  // than the robot loop period (~20ms)!
  // That means this value should be set considerably lower to account for delays caused by other
  // events.
  // FIXME: Do this the right way so we don't have to restrict the timing
  private static final int LOOP_PERIOD_MS = 5;

  private static EventWatcherThread instance;

  private final Object queueLock = new Object();
  private ArrayList<Event> queue;
  private ArrayList<Event> pruneList; // Needed to avoid ConcurrentModificationException!

  public static synchronized EventWatcherThread getInstance() {
    if (instance == null) {
      instance = new EventWatcherThread();
      instance.start();
    }

    return instance;
  }

  private EventWatcherThread() {
    queue = new ArrayList<>();
    pruneList = new ArrayList<>();
  }

  @Override
  public void run() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        for (Event e : queue) {
          if (e.predicate()) {
            e.run();

            if (e.pruneMe()) {
              pruneList.add(e);
            }
          }
        }

        for (Event e : pruneList) {
          queue.remove(e);
        }

        Thread.sleep(LOOP_PERIOD_MS);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }
  }

  public void registerEvent(Event e) {
    synchronized (queueLock) {
      queue.add(e);
    }
  }

  public void unRegisterEvent(Event e) {
    synchronized (queueLock) {
      queue.remove(e);
    }
  }

  public void resetQueue() {
    synchronized (queueLock) {
      queue.clear();
    }
  }
}
