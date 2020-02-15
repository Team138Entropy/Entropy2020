package frc.robot.subsystems.auto.path;

import javax.annotation.Nullable;
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
    return Optional.ofNullable(paths.get(pathName));
  }
}
