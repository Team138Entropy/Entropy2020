package frc.robot.subsystems.auto.path;

import frc.robot.Logger;

public abstract class Segment {
  protected static Logger logger = Path.getLogger();

  /** Any initialization that needs to be done before the segment runs should be done here. */
  public abstract void init();

  /** Called periodically. Do most of your shit here. */
  public abstract void tick();

  /**
   * Tell the caller whether or not this segment has completed. It is safe to assume this will run
   * after every tick.
   *
   * @return whether or not the segment has completed.
   */
  public abstract boolean finished();

  /**
   * Returns a new Segment created with the same parameters. TODO: Find a better way to do this
   *
   * @return a copy of this segment.
   */
  public abstract Segment copy();
}
