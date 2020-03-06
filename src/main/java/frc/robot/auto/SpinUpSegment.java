package frc.robot.auto;

import frc.robot.Robot;

public class SpinUpSegment extends Segment {

  public SpinUpSegment() {}

  @Override
  public void init() {}

  @Override
  public void tick() {
    Robot.setSpinningUp(true);
  }

  @Override
  public boolean finished() {
    return true;
  }

  @Override
  public Segment copy() {
    return new SpinUpSegment();
  }
}
