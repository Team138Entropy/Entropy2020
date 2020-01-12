package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends Subsystem {

    // TODO: Integrate with other subsystems for real
    // TEMPORARY STUFF BEGINS HERE
    private static final int ROLLER_PORT = 0;
    private static final double ROLLER_SPEED = 0.5;
    private static final double FIRE_DELAY_SECONDS = 0.5;

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

    // Aggregation
    private Shooter instance;
    private WPI_TalonSRX m_roller;
    private Turret m_turret;
    private Vision m_vision;
    private Timer m_timer;

    // State variables
    private boolean m_ableToFire = false;
    private boolean m_fire = false;

    public Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private Shooter() {
        m_roller = new WPI_TalonSRX(ROLLER_PORT);

        // TODO: Replace these with real subsystems
        m_turret = position -> System.out.println(
            "Setting dummy turret position to ("
            + position.getHorizontal()
            + ", "
            + position.getVertical()
            + ")"
        );
        m_vision = () -> {
            System.out.println("Getting dummy vision target");
            return new TurretPosition(0, 0);
        };

        m_timer = new Timer();
    }

    // Call this in the robot loop.
    // TODO: Consider replacing this home-grown timing system with WPILib's scheduler.
    public void periodic() {

    }

    public void fireSingle() {
//        if (!m_ableToFire) {
//            m_timer.reset();
//            m_timer.start();
//
//        }
    }

    public void fireAuto() {}

    public void stopAuto() {}

    public void target() {
        m_turret.set(m_vision.calcTargetPosition());
    }

    private void start() {
        m_roller.set(ControlMode.PercentOutput, ROLLER_SPEED);
    }

    private void stop() {
        m_roller.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void ZeroSensors() {}

    @Override
    public void CheckSubsystems() {}
}
