package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents an event to be checked for by the {@link EventWatcherThread EventWatcherThread}.
 */
public interface Event {
    /**
     * Check whether or not some command should be added to the scheduler.
     * Try to keep this as short-running as possible.
     * TODO timeout?
     *
     * @return Whether or not it should be added to the scheduler.
     */
    boolean check();

    /**
     * Get the command to add to the scheduler.
     *
     * @return The command to add to the scheduler.
     */
    Command getCommand();
}
