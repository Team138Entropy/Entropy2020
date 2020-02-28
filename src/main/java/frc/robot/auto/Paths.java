package frc.robot.auto;

import java.util.HashMap;
import java.util.Optional;

public class Paths {
  private static HashMap<String, Path> paths = new HashMap<>();
  public static final Path NO_OP = new Path();

  static {
    paths.put("test", new Path()
      .append(new DriveSegment(5))
      .append(new ShootSegment())
      .append(new DriveSegment(-5))
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
