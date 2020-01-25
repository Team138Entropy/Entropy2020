package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.StartBallStorage;
import frc.robot.subsystems.Storage;
import frc.robot.Logger;

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

