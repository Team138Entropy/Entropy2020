package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Storage;

public class StopBallStorage extends InstantCommand {

  public StopBallStorage() {}

  public void execute() {
    Storage.getInstance().stop();
  }
}
