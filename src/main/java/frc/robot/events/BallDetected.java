package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Logger;
import frc.robot.commands.StartBallStorage;
import frc.robot.subsystems.Storage;

public class BallDetected implements Event {

  Logger mLogger = new Logger("ballDetected");

  public boolean check() {
   // return (Storage.getInstance().isBallDetected());
   return false;
  }

  public Command getCommand() {
    mLogger.log("Ball Detected");
    return new StartBallStorage();
  }
}
