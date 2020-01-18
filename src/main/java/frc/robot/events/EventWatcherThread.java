package frc.robot.events;

import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.ArrayList;
import java.util.LinkedHashMap;

/**
 * Singleton thread for monitoring things.
 */
public class EventWatcherThread extends Thread {

    private ArrayList<Event> queue = new ArrayList<>();
    private LinkedHashMap<Event, Boolean> lastStateCache = new LinkedHashMap<>();

    private static EventWatcherThread thread = new EventWatcherThread();

    public static EventWatcherThread getInstance() {
        return thread;
    }

    @Override
    public void run() {

        //noinspection InfiniteLoopStatement
        while (true) {
            for (Event e : queue) {

                // We have to store this because e.check() might change
                boolean savedState = e.check();

                // Add the event to the cache if it wasn't already there
                if (!lastStateCache.containsKey(e)) {
                  lastStateCache.put(e, savedState);
                }

                // Add the command to the scheduler only if the state of the event changed
                if (savedState && !lastStateCache.get(e)) {
                  Scheduler.getInstance().add(e.getCommand());
                }

                lastStateCache.replace(e, savedState);
            }
        }
    }

    public void addEvent(Event e) {
        if (!queue.contains(e)) {
            queue.add(e);
            System.out.println("Event added (" + queue.size() + " total)");
        }
    }
}
