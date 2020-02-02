package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class ShuffleboardHandler extends Subsystem {
  private static ShuffleboardHandler mInstance;

  public static synchronized ShuffleboardHandler getInstance() {
    if (mInstance == null) {
      mInstance = new ShuffleboardHandler();
    }
    return mInstance;
  }

  // In the past, this class was used for retrieval of numbers from the Shuffleboard
  // SmartDashboard and not sending data. Stuff used in number tuning usually goes here.
  public void UpdateShuffleboard() {}

  public void zeroSensors() {}

  public void checkSubsystem() {}
}
