package frc.robot.subsystems.auto.path;

import frc.robot.Logger;

public abstract class Segment {
  protected static Logger logger = Path.getLogger();

  /** Any initialization that needs to be done before the segment runs should be done here. */
  public abstract void init();

  /** Called periodically. Do most of your shit here. */
  public abstract void tick();

  /**
   * Tell the caller whether or not this segment has completed.
   *
   * @return whether or not the segment has completed.
   */
  public abstract boolean finished();
}
