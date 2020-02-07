package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Logger;
import frc.robot.commands.StopBallStorage;
import frc.robot.subsystems.Intake;

public class BallStored implements Event {
  Logger mLogger;

  public BallStored() {
    mLogger = new Logger("ballStored");
  }

  public boolean check() {
    return (!Intake.getInstance().isBallDetected());
  }

  public Command getCommand() {
    mLogger.info("Ball Stored");
    return new StopBallStorage();
  }
}
