package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.StopRoller;
import frc.robot.subsystems.Storage;

public class BallStored implements Event {


    public boolean check() {
        return (Storage.getInstance().isBallStored());
    }

    public Command getCommand() {
        System.out.println("Ball Stored");
        return new StopRoller();
    }

    
}
