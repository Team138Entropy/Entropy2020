package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopRoller extends InstantCommand {

    public StopRoller() {
      
    }

    public void execute() {
        Storage.getInstance().stop();
    }
}
