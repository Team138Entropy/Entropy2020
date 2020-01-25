package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Storage;

public class StartBallStorage extends InstantCommand {

  public StartBallStorage() {}

  public void execute() {
    Storage.getInstance().storeBall();
  }
}
