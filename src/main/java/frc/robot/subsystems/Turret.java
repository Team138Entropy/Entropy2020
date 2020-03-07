package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class Turret extends Subsystem {
  private static Turret sInstance;

  private final WPI_TalonSRX mTurretTalon;
  private final double TicksPerDegree = Constants.kTicksPerDegee;

  // Home Position of Turret
  //  as vision is disabled, goes to homing to get back home
  private final double HomePosition = 0;

  // How Close the Turret can be
  private final double AutoAimDeadband = 0.3;

  enum TurretState {
    AUTO_AIM,
    HOME,
    SEARCHING,
    MANUAL_AIM
  };

  // Default to Aiming State
  // this turret state should remain local to the turret class
  private TurretState mCurrentState = TurretState.AUTO_AIM;

  protected PeriodicIO mPeriodicIO = new PeriodicIO();

  // Class of values that are periodically updated
  public static class PeriodicIO {
    // Inputs
    public double timestamp;
    public double CurrentPosition;
    public double Velocity;

    // Outputs
    public double demand; // motor output, could be a position, or percent
    public double angle;
    public double feedforward;
  };

  public static Turret getInstance() {
    if (sInstance == null) {
      sInstance = new Turret();
    }

    return sInstance;
  }

  /** Set up our talon, logger and potentiometer */
  private Turret() {
    mTurretTalon = new WPI_TalonSRX(Constants.Talon_Turret);
    mTurretTalon.configFactoryDefault();
    mTurretTalon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
    mTurretTalon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
    mTurretTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    mTurretTalon.config_kF(0, 0); // MUST BE 0 in Position mode
    mTurretTalon.config_kP(0, .3);
    mTurretTalon.config_kI(0, 0);
    mTurretTalon.config_kD(0, 0);
    mTurretTalon.config_IntegralZone(0, 50);
    mTurretTalon.setNeutralMode(NeutralMode.Brake);
  }

  // peridocally read inputs
  @Override
  public synchronized void readPeriodicInputs() {
    // store current encoder position
    mPeriodicIO.CurrentPosition = mTurretTalon.getSelectedSensorPosition();
    mPeriodicIO.Velocity = mTurretTalon.getSelectedSensorVelocity();
  }

  // periodically write outputs
  @Override
  public synchronized void writePeriodicOutputs() {

    // Control Turret Based on State
    if (mCurrentState == TurretState.AUTO_AIM) {
      // Perform Auto Aim!
      // deadband: Angle error must be greater than 1 degree
      if (Math.abs(mPeriodicIO.angle) > AutoAimDeadband) {
        mTurretTalon.set(ControlMode.Position, mPeriodicIO.demand);
      }

    } else if (mCurrentState == TurretState.HOME) {
      // going to home position
      mTurretTalon.set(ControlMode.Position, HomePosition);

      // if at home position go to manual
      if (Math.abs(HomePosition - Math.abs(mPeriodicIO.CurrentPosition)) <= 550) {
        // Home enough.. go to manual
        mCurrentState = TurretState.MANUAL_AIM;
      }

    } else if (mCurrentState == TurretState.SEARCHING) {
      // searching for a track
      // TODO
      // move in between two areas

    } else if (mCurrentState == TurretState.MANUAL_AIM) {
      // Manual Control
      mTurretTalon.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }
  }

  // Return the turret to its home position
  // home position is on the backside of the robot
  public synchronized void ReturnHome() {
    // set to home state
    mCurrentState = TurretState.HOME;
  }

  // Operator Driven Manual Control
  public synchronized void SetManualOutput(double value) {
    // detect change from Auto to Manual
    // this indicates that we rotate home
    if (mCurrentState == TurretState.AUTO_AIM) {
      // set to home
      ReturnHome();
      return;
    }

    // buisness as usual..
    mPeriodicIO.demand = value;
    // Force correct control mode
    if (mCurrentState != TurretState.MANUAL_AIM) {
      // change pid slot if needed
      mCurrentState = TurretState.MANUAL_AIM;
    }
  }

  // Vision Aim System
  public synchronized void SetAimError(double angle) {

    double velocity = mPeriodicIO.Velocity;
    velocity = Math.abs(velocity);

    // Shooter is offset
    if (velocity <= Constants.TURRET_AIM_MAX_VELOCITY) {
      mPeriodicIO.angle = angle;
      double setpoint = mPeriodicIO.CurrentPosition - (angle * TicksPerDegree);
      mPeriodicIO.demand = setpoint;
      mPeriodicIO.feedforward = 0;

      if (mCurrentState != TurretState.AUTO_AIM) {
        // change pid slot if needed
        mCurrentState = TurretState.AUTO_AIM;
      }
    }
  }

  // zero sensors
  public synchronized void zeroSensors() {
    mTurretTalon.setSelectedSensorPosition(0);
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem() {}
}
