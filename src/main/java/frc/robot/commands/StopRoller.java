package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Storage;

public class StopRoller extends InstantCommand {

    public StopRoller() {}

    public void execute() {
        Storage.getInstance().stop();
    }
}
