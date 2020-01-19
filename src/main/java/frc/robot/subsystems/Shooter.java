package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;

/** Singleton that represents the shooter mechanism. */
public class Shooter extends Subsystem {

    // TODO: Integrate with other subsystems for real
    // TEMPORARY STUFF BEGINS HERE
    private static final int ROLLER_PORT = 0;
    private static final int MAX_CAPACITY = 5;

    // TODO: Tune these values
    private static final double ROLLER_SPEED = 1;
    private static final double SPINUP_DELAY_SECONDS = 0.5;
    private static final double FIRE_DURATION_SECONDS = 0.5;

    private static class TurretPosition {
        private double m_azimuth, m_elevation;

        public TurretPosition(double azimuth, double elevation) {
            m_azimuth = azimuth;
            m_elevation = elevation;
        }

        public double getAzimuth() {
            return m_azimuth;
        }

        public double getElevation() {
            return m_elevation;
        }
    }

    @FunctionalInterface
    private interface Turret {
        void set(TurretPosition position);
    }

    @FunctionalInterface
    private interface Vision {
        TurretPosition calcTargetPosition();
    }

    @FunctionalInterface
    private interface Intake {
        void shoveANodeIntoTheThing();
    }

    // TEMPORARY STUFF ENDS HERE

    // Aggregation
    private static Shooter instance;
    private WPI_TalonSRX m_roller;
    private Turret m_turret;
    private Vision m_vision;
    private Intake m_intake;

    // State variables
    public enum State {
        IDLE,
        FULL_SPEED,
        SPINNING_UP,
        FIRING
    }

    private State state;
    private int m_buffer = 0;
    private Timer m_spinUpTimer;
    private Timer m_fireTimer;

    public static synchronized Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    public State getState() {
        return state;
    }

    private Shooter() {
        m_roller = new WPI_TalonSRX(ROLLER_PORT);

        // TODO: Replace these with real subsystems
        m_turret =
                position ->
                        System.out.println(
                                "Setting dummy turret position to ("
                                        + position.getAzimuth()
                                        + ", "
                                        + position.getElevation()
                                        + ")");
        m_vision =
                () -> {
                    System.out.println("Getting dummy vision target");
                    return new TurretPosition(0, 0);
                };
        m_intake = () -> System.out.println("Shoving a ball into the thing");

        m_spinUpTimer = new Timer();
        m_fireTimer = new Timer();
    }

    /**
     * Call this in the robot loop. TODO: Consider replacing this home-grown timing system with
     * WPILib's scheduler.
     */
    public void periodic() {

        // Check if we're done spinning up yet
        if (state == State.SPINNING_UP && m_spinUpTimer.get() >= SPINUP_DELAY_SECONDS) {
            state = State.FULL_SPEED;
            m_spinUpTimer.stop();
            m_spinUpTimer.reset();
        }

        // Check if we're done firing yet
        if (state == State.FIRING && m_fireTimer.get() >= FIRE_DURATION_SECONDS) {
            state = State.IDLE;
            m_fireTimer.stop();
            m_fireTimer.reset();
        }

        // Handle buffered fire operations
        if (m_buffer > 0) {

            // If we haven't started spinning up yet
            if (state != State.SPINNING_UP) {
                start();
                m_spinUpTimer.reset();
                m_spinUpTimer.start();
            }

            if (state == State.FULL_SPEED) {
                state = State.FIRING;
                m_intake.shoveANodeIntoTheThing();
                m_fireTimer.start();
                m_buffer--;
            }
        } else {
            // Handle the case where the buffer was reset while we were doing something
            if (state == State.SPINNING_UP || state == State.FULL_SPEED) {
                stop();
                m_spinUpTimer.stop();
                m_spinUpTimer.reset();
            }
        }
    }

    /** Buffers another fire operation. */
    public void fireSingle() {
        m_buffer++;
    }

    /**
     * Equivalent to calling {@link #resetBuffer()} and then calling {@link #fireSingle()} a number
     * of times equal to the number of balls in the storage mechanism.
     */
    public void fireAuto() {
        resetBuffer();
        for (int i = 0; i < MAX_CAPACITY; i++) {
            fireSingle();
        }
    }

    /**
     * Resets the firing buffer. Has the effect of cancelling any buffered fire operations,
     * including automatic fire.
     */
    public void resetBuffer() {
        m_buffer = 0;
    }

    /** The same as {@link #resetBuffer()}. Exists to make calling code more declarative. */
    public void stopFiring() {
        resetBuffer();
    }

    /** Tells the turret to move to where the vision system says we should be. */
    public void target() {
        m_turret.set(m_vision.calcTargetPosition());
    }

    /** Starts the roller. */
    private void start() {
        m_roller.set(ControlMode.PercentOutput, ROLLER_SPEED);
    }

    /** Stops the roller. */
    private void stop() {
        m_roller.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void ZeroSensors() {}

    @Override
    public void CheckSubsystems() {}
}
