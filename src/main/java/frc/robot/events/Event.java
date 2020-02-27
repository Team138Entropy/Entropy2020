package frc.robot.events;

public interface Event extends Runnable {

  /**
   * The predicate for the main logic of this event.
   * @return whether or not {@link #run()} should be called this tick.
   */
  boolean predicate();

  /**
   * Tells the caller that {@link #predicate()} will never return true again.
   * @return whether or not the event should be pruned.
   */
  default boolean pruneMe() { return false; }
}
