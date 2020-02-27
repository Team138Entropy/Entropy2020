package frc.robot.events;

/**
 * An event to be monitored by the {@link EventWatcherThread}.
 */
public interface Event extends Runnable {

  /**
   * The predicate for the main logic of this event.
   *
   * @return whether or not {@link #run()} should be called this tick.
   */
  boolean predicate();

  /**
   * Tells the caller that the event should be pruned.
   *
   * @return whether or not the event should be pruned.
   */
  default boolean pruneMe() {
    return false;
  }
}
