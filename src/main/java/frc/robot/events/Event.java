package frc.robot.events;

/** An event to be monitored by the {@link EventWatcherThread}. */
public interface Event extends Runnable {

  /**
   * The predicate for the main logic of this event. Any logic that needs to be run every tick
   * should go here.
   *
   * @return whether or not {@link #run()} should be called this tick.
   */
  boolean predicate();

  /**
   * Tells the caller that the event should be pruned. It is safe to assume that {@link
   * #predicate()} and {@link #run()} have been called at least once at this point.
   *
   * @return whether or not the event should be pruned.
   */
  default boolean pruneMe() {
    return false;
  }
}
