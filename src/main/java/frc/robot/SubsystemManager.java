package frc.robot;

import frc.robot.subsystems.Subsystem;
import java.util.ArrayList;
import java.util.List;

public class SubsystemManager {
  Logger mSubsystemLogger;
  public static SubsystemManager mInstance = null;

  private List<Subsystem> mSubsystems;

  public static synchronized SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }
    return mInstance;
  }

  private SubsystemManager() {
    mSubsystemLogger = new Logger("subsystem");mSubsystems = new ArrayList<>();
  }

  /*
      Add Subsystem to the Subsystem manager
  */
  public void RegisterSubsystem(Subsystem arg) {
    mSubsystems.add(arg);
  }

  /*
      Zero all Sensors
  */
  public void ZeroSensors() {
    for (int i = 0; i < mSubsystems.size(); i++) {
      try {
        mSubsystems.get(i).ZeroSensors();
      } catch (Exception e) {
        mSubsystemLogger.verbose("Sensor Zero Exception: " + e.getMessage());
      }
    }
  }

  /*
      Run Subsystem check command on all subsystems
  */
  public void CheckSubsystems() {
    for (int i = 0; i < mSubsystems.size(); i++) {
      try {
        mSubsystems.get(i).CheckSubsystems();
      } catch (Exception e) {
        mSubsystemLogger.verbose("Subsystem Check Exception: " + e.getMessage());
      }
    }
  }
}
