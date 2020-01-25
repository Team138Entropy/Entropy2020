package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Logger;
import frc.robot.commands.StartBallStorage;
import frc.robot.subsystems.Storage;

public class BallDetected implements Event {

  Logger mLogger = new Logger("Ball Detector");

  public boolean check() {
    return (Storage.getInstance().isBallDetected());
  }

  public Command getCommand() {
    mLogger.log("Ball Detected");
    return new StartBallStorage();
  }
}
