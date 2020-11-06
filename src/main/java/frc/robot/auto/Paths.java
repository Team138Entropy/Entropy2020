package frc.robot.auto;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Optional;

public class Paths {
  private static HashMap<String, Path> paths = new HashMap<>();
  public static final Path NO_OP = new Path();

  static {
    paths.put(
        "comp1",
        new Path()
            .append(new VisionToggleSegment())
            .append(
                new DriveSegment(
                    40.0 / 12.0, Constants.DEFAULT_CRUISE_VELOCITY, Constants.DEFAULT_ACCEL))
            .append(new VisionToggleSegment()));

    paths.put(
        "comp2",
        new Path()
            .append(new VisionToggleSegment())
            .append(new SpinUpSegment())
            .append(
                new DriveSegment(
                    40.0 / 12.0, Constants.DEFAULT_CRUISE_VELOCITY, Constants.DEFAULT_ACCEL))
            .append(new ShootSegment()));

    // paths.put(
    //     "comp3",
    //     new Path()
    //         .append(
    //             new DriveSegment(10, Constants.DEFAULT_CRUISE_VELOCITY,
    // Constants.DEFAULT_ACCEL)));

    // paths.put(
    //     "comp4",
    //     new Path()
    //         .append(new SpinUpSegment())
    //         .append(
    //             new DriveSegment
    //                 13.5 / 12, Constants.DEFAULT_CRUISE_VELOCITY, Constants.DEFAULT_ACCEL))
    //         .append(new VisionToggleSegment())
    //         .append(new ShootSegment())
    //         .append(new VisionToggleSegment())
    //         .append(new IntakeSegment(3))
    //         .append(new DriveSegment(86.63 / 12, 300, Constants.DEFAULT_ACCEL))
    //         .append(new SyncIntakeSegment())
    //         .append(new VisionToggleSegment())
    //         .append(new ShootSegment())
    //         .append(new VisionToggleSegment())
    //         .append(
    //             new DriveSegment(1, Constants.DEFAULT_CRUISE_VELOCITY,
    // Constants.DEFAULT_ACCEL)));

    paths.put(
        "comp4",
        new Path()
            .append(new SpinUpSegment())
            .append(
                new DriveSegment(
                    47.0 / 12.0, Constants.DEFAULT_CRUISE_VELOCITY, Constants.DEFAULT_ACCEL))
            .append(new VisionToggleSegment())
            .append(new ShootSegment())
            .append(new VisionToggleSegment())
            .append(new IntakeSegment(3))
            .append(new DriveSegment(86.0 / 12.0, 300, Constants.DEFAULT_ACCEL))
            .append(new SyncIntakeSegment())
            .append(new VisionToggleSegment())
            .append(new ShootSegment())
            .append(new VisionToggleSegment()));
  }

  /**
   * Finds a path by name.
   *
   * @param pathName the name of the path.
   * @return a copy of the path if one with a matching name was found.
   */
  public static Optional<Path> find(String pathName) {
    Path path = paths.get(pathName);

    if (path == null) {
      return Optional.empty();
    } else {
      // We probably don't want to modify the original path
      return Optional.of(path.copy());
    }
  }
}
