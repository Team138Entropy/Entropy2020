package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends Subsystem {

    // TODO: Integrate with other subsystems for real
    // TEMPORARY STUFF BEGINS HERE
    private static final int ROLLER_PORT = 0;

    private static class TurretPosition {
        private double m_horizontal, m_vertical;

        public TurretPosition(double horizontal, double vertical) {
            m_horizontal = horizontal;
            m_vertical   = vertical;
        }

        public double getHorizontal() { return m_horizontal; }
        public double getVertical() { return m_vertical; }
    }

    @FunctionalInterface
    private interface Turret {
        void set(TurretPosition position);
    }

    @FunctionalInterface
    private interface Vision {
        TurretPosition calcTargetPosition();
    }

    // TEMPORARY STUFF ENDS HERE

    private Shooter instance;
    private WPI_TalonSRX m_roller;
    private Turret m_turret;
    private Vision m_vision;

    public Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private Shooter() {
        m_roller = new WPI_TalonSRX(ROLLER_PORT);

        // Replace these with real subsystems
        m_turret = position -> {};
        m_vision = () -> new TurretPosition(0, 0);
    }

    public void fireSingle() {

    }

    public void fireAuto() {}

    public void stopAuto() {}

    public void target() {}



    @Override
    public void ZeroSensors() {}

    @Override
    public void CheckSubsystems() {}
}
