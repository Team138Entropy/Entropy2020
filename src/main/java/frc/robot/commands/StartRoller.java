package frc.robot.commands;

import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class StartRoller extends InstantCommand {

    public StartRoller() {
        
    }

    public void execute() {
        Storage.getInstance().start();
    }
}
