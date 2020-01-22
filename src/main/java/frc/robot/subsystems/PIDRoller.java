package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

class PIDRoller {

    private static final int pidLoopIndex = 0;
    private static final int timeoutMs = 10;

    private WPI_TalonSRX talon;

    PIDRoller(int talonPort, double p, double i, double d) {
        super();

        talon = new WPI_TalonSRX(talonPort);

        // All of this was ripped from the 2019 elevator code
		talon.configNominalOutputForward(0, timeoutMs);
		talon.configNominalOutputReverse(0, timeoutMs);
		talon.configPeakOutputForward(1, timeoutMs);
        talon.configPeakOutputReverse(-1, timeoutMs);
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidLoopIndex, timeoutMs);
        talon.setSensorPhase(false);

        talon.configAllowableClosedloopError(pidLoopIndex, 0, timeoutMs);

        talon.config_kP(pidLoopIndex, p);
        talon.config_kI(pidLoopIndex, i);
        talon.config_kD(pidLoopIndex, d);

        talon.setNeutralMode(NeutralMode.Coast);

        talon.config_IntegralZone(pidLoopIndex, 200, timeoutMs);
    }

    void setSpeed(int posPer100Ms) {
        talon.set(ControlMode.Velocity, posPer100Ms);
    }
}
