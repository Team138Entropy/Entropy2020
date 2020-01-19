package frc.robot.events;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.StartRoller;
import frc.robot.subsystems.Storage;

public class BallDetected implements Event {

    public boolean check() {
        return (Storage.getInstance().isBallDetected());
    }

    public Command getCommand() {
        System.out.println("Ball Detected");
        return new StartRoller();
    }
}
