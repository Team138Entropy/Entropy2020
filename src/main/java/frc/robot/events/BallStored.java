package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Logger;
import frc.robot.commands.StopBallStorage;
import frc.robot.subsystems.Storage;

public class BallStored implements Event {
  Logger mLogger;

  public BallStored() {
    mLogger = new Logger("ballStored");
  }

  public boolean check() {
    return (Storage.getInstance().isBallStored());
  }

  public Command getCommand() {
    mLogger.info("Ball Stored");
    Storage.getInstance().increaseBallCount();
    return new StopBallStorage();
  }
}
