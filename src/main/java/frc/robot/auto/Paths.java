package frc.robot.auto;

import java.util.HashMap;
import java.util.Optional;

import frc.robot.subsystems.Drive;

public class Paths {
  private static HashMap<String, Path> paths = new HashMap<>();
  public static final Path NO_OP = new Path();

  static {
    paths.put("test", new Path()
      .append(new GyroTurnSegment(180))
    // .append(new ShootSegment())
      // .append(new IntakeSegment(3))
      // .append(new DriveSegment(6, 250, 125))
      // .append(new AsyncShootSegment())
      // .append(new DriveSegment(-6, Drive.DEFAULT_CRUISE_VELOCITY, Drive.DEFAULT_ACCEL))
      // .append(new GyroTurnSegment(180))
      // .append(new GyroTurnSegment(180))
    );
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
