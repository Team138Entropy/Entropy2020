package frc.robot.events;

import java.util.ArrayList;

public class EventWatcherThread extends Thread {
  private static final int LOOP_PERIOD_MS = 20;

  private static EventWatcherThread instance;

  private ArrayList<Event> queue;

  public static synchronized EventWatcherThread getInstance() {
    if (instance == null) {
      instance = new EventWatcherThread();
    }

    return instance;
  }

  private EventWatcherThread() {
    this.queue = new ArrayList<>();
  }

  @Override
  public void run() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        for (Event e : queue) {
          if (e.predicate()) {
            e.run();
          }

          if (e.pruneMe()) {
            queue.remove(e);
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
