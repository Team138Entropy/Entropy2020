package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem {

  /** Aggregation */
  private static Climber instance;

  private Timer mClimbingTimer;

  public Climber() {
    mClimbingTimer = new Timer();
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public void extend(double timeMilliseconds) {
    mClimbingTimer.start();
  }

  public void retract(double timeMilliseconds) {
    mClimbingTimer.start();
  }

  public boolean isReadyForNextState (double time) {
    return mClimbingTimer.get() >= time;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
