package frc.robot;

import frc.robot.subsystems.Subsystem;
import frc.robot.util.loops.*;
import java.util.ArrayList;
import java.util.List;

public class SubsystemManager implements ILooper {
  Logger mSubsystemLogger;
  public static SubsystemManager mInstance = null;

  private List<Subsystem> mSubsystems;

  // store all defined loops
  private List<Loop> mLoops = new ArrayList<>();

  // Runs Subsystems Enabled Related Loops
  private class EnabledLoop implements Loop {
    @Override
    public void onStart(double timestamp) {
      mLoops.forEach(l -> l.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
      mSubsystems.forEach(Subsystem::readPeriodicInputs);
      mLoops.forEach(l -> l.onLoop(timestamp));
      mSubsystems.forEach(Subsystem::writePeriodicOutputs);
    }

    @Override
    public void onStop(double timestamp) {
      mLoops.forEach(l -> l.onStop(timestamp));
    }
  }

  // Runs Subsystems Disabled related loops
  private class DisabledLoop implements Loop {
    @Override
    public void onStart(double timestamp) {}

    @Override
    public void onLoop(double timestamp) {
      mSubsystems.forEach(Subsystem::readPeriodicInputs);
    }

    @Override
    public void onStop(double timestamp) {}
  }

  public static synchronized SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }
    return mInstance;
  }

  private SubsystemManager() {
    mSubsystemLogger = new Logger("subsystem");
    mSubsystems = new ArrayList<>();
  }

  /*
      Add Subsystem to the Subsystem manager
  */
  public void registerSubsystem(Subsystem arg) {
    mSubsystems.add(arg);
  }

  /*
  Register the Enabled Looper
  Looper that periodically runs in the background to polls elements
  Registers any enabled loops a subsystem might have
  Basically points to the location of our looper
  */
  public void registerEnabledLoops(Looper enabledLooper) {
    mSubsystems.forEach(s -> s.registerEnabledLoops(this));
    enabledLooper.register(new EnabledLoop());
  }

  public void registerDisabledLoops(Looper disabledLooper) {
    disabledLooper.register(new DisabledLoop());
  }

  @Override
  public void register(Loop loop) {
    mLoops.add(loop);
  }

  /*
      Zero all Sensors
  */
  public void zeroSensors() {
    for (int i = 0; i < mSubsystems.size(); i++) {
      try {
        mSubsystems.get(i).zeroSensors();
      } catch (Exception e) {
        mSubsystemLogger.verbose("Sensor Zero Exception: " + e.getMessage());
      }
    }
  }

  /*
      Run Subsystem check command on all subsystems
  */
  public void checkSubsystems() {
    for (int i = 0; i < mSubsystems.size(); i++) {
      try {
        mSubsystems.get(i).checkSubsystem();
      } catch (Exception e) {
        mSubsystemLogger.verbose("Subsystem Check Exception: " + e.getMessage());
      }
    }
  }
}
