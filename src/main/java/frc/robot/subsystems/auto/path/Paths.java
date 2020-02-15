package frc.robot.subsystems.auto.path;

import java.util.HashMap;
import java.util.Optional;

public class Paths {
  private static HashMap<String, Path> paths = new HashMap<>();
  public static final Path NO_OP = new Path();

  static {
    paths.put("test",
        new Path()
            .append(new DriveSegment(2))
            .append(new TurnSegment(45))
            .append(new DriveSegment(1))
    );
  }

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
