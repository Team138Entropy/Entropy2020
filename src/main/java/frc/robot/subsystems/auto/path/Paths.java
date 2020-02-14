package frc.robot.subsystems.auto.path;

public class Paths {
  public static Path testPath;

  static {
    testPath = new Path()
      .append(new DriveSegment(2))
      .append(new TurnSegment(45))
      .append(new DriveSegment(1));
  }
}
