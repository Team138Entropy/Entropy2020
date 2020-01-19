package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;

class PIDRoller extends Subsystem {

    private WPI_TalonSRX talon;
    private PIDController pidController;

    PIDRoller(int talonPort, double p, double i, double d) {
        super();

        talon = new WPI_TalonSRX(talonPort);
        pidController = new PIDController(p, i, d);
    }

    @Override
    public void ZeroSensors() {}

    @Override
    public void CheckSubsystems() {}
}
