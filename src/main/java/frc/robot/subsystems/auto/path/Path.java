package frc.robot.subsystems.auto.path;

import frc.robot.Logger;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 * Represents a path to be followed.
 */
public class Path {
  private LinkedList<Segment> segments;
  private ArrayList<Segment> uninitializedSegments;
  private static Logger logger = new Logger("Path");

  protected static Logger getLogger() {
    return logger;
  }

  public Path() {
    segments = new LinkedList<>();
    uninitializedSegments = new ArrayList<>();
  }

  /**
   * Appends a new segment to the path.
   * @param segment the segment to be added.
   * @return the path object, so that these calls can be chained together.
   */
  public synchronized Path append(Segment segment) {
    segments.add(segment);
    uninitializedSegments.add(segment);
    return this;
  }

  private @Nullable Segment getCurrentSegment() {
    return segments.peekFirst();
  }

  /**
   * Does the stuff. Call this periodically.
   */
  public void tick() {
    Segment segment = getCurrentSegment();
    if (segment != null) {
      if (!uninitializedSegments.contains(segment)) {
        segment.init();
        uninitializedSegments.remove(segment);
      }

      segment.tick();

      if (segment.finished()) {
        segments.removeFirst();
      }
    }
  }

}
