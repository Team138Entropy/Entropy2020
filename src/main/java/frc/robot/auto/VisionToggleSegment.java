package frc.robot.auto;

public class VisionToggleSegment extends Segment {

  public VisionToggleSegment() {}

  private static boolean enabled = false;

  @Override
  public void init() {
    enabled = true;
  }

  @Override
  public void tick() {}

  public static boolean getToggle() {
    if (enabled) {
      enabled = false;
      return true;
    }
    return false;
  }

  @Override
  public boolean finished() {
    return true;
  }

  @Override
  public Segment copy() {
    return new VisionToggleSegment();
  }
}
