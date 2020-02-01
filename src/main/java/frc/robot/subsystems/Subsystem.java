package frc.robot.subsystems;

import frc.robot.SubsystemManager;

/*
All Subsystem classes must extend the subsystem abstract class
This will allow us to call common methods from the subsystem management
the constructor will automatically add to the subsystem manager
*/
public abstract class Subsystem {

  public Subsystem() {
    SubsystemManager.getInstance().RegisterSubsystem(this);
  }

  public abstract void zeroSensors();

  public abstract void checkSubsystems();
}
