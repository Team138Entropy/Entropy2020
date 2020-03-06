package frc.robot.auto;

import frc.robot.Logger;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 * A queue of {@link Segment}s to be executed sequentially. Intended to be used for autonomous
 * driving, but could be used for any sequence of tasks. Similar to WPILib's {@link
 * edu.wpi.first.wpilibj2.command.SequentialCommandGroup SequentialCommandGroup}.
 *
 * @see Segment
 */
public class Path {
  private LinkedList<Segment> segments;
  private ArrayList<Segment> uninitializedSegments;
  private boolean finished = false;

  private static Logger logger = new Logger("path");

  protected static Logger getLogger() {
    return logger;
  }

  public Path() {
    segments = new LinkedList<>();
    uninitializedSegments = new ArrayList<>();
  }

  /**
   * Appends a new segment to the path.
   *
   * @param segment the segment to be added.
   * @return the path object, so that these calls can be chained together.
   */
  public synchronized Path append(Segment segment) {
    segments.add(segment);
    uninitializedSegments.add(segment);
    return this;
  }

  private Segment getCurrentSegment() {
    return segments.peekFirst();
  }

  /** Does the stuff. Call this periodically. */
  public void tick() {
    if (!finished) {
      Segment segment = getCurrentSegment();
      if (segment != null) {
        if (uninitializedSegments.contains(segment)) {
          logger.info("Segments: " + segments.toString());
          segment.init();
          uninitializedSegments.remove(segment);
        }

        segment.tick();

        if (segment.finished()) {
          logger.info("Segment finished");
          segments.removeFirst();
        }
      } else {
        logger.info("Path finished");
        finished = true;
      }
    }
  }

  /**
   * Returns a copy of this path.
   *
   * @return the copy.
   */
  public Path copy() {
    Path p = new Path();
    for (Segment s : segments) {
      p.append(s.copy());
    }
    return p;
  }
}
