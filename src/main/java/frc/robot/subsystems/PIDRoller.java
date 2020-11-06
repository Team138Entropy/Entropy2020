package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Robot;

class PIDRoller {

  private final int PID_LOOP_INDEX = 0;
  private final int TIMEOUT_MS = 10;

  private final WPI_TalonSRX mTalon;
  private final WPI_TalonSRX mTalonSlave;

  PIDRoller(int talonPort, int talon2Port, double p, double i, double d, double f) {
    mTalon = new WPI_TalonSRX(talonPort);
    mTalonSlave = new WPI_TalonSRX(talon2Port);

    mTalon.configFactoryDefault();
    mTalonSlave.configFactoryDefault();

    // All of this was ripped from the 2019 elevator code
    mTalon.configNominalOutputForward(0, TIMEOUT_MS);
    mTalon.configNominalOutputReverse(0, TIMEOUT_MS);
    mTalon.configPeakOutputForward(1, TIMEOUT_MS);
    mTalon.configPeakOutputReverse(-1, TIMEOUT_MS);

    mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 50);
    mTalon.configVelocityMeasurementWindow(64);
    mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    mTalon.setSensorPhase(!Robot.getIsPracticeBot());

    // mTalon.enableVoltageCompensation(true); // turn on/off feature
    mTalon.configAllowableClosedloopError(PID_LOOP_INDEX, 50, TIMEOUT_MS);

    mTalon.config_kP(PID_LOOP_INDEX, p);
    mTalon.config_kI(PID_LOOP_INDEX, i);
    mTalon.config_kD(PID_LOOP_INDEX, d);
    mTalon.config_kF(PID_LOOP_INDEX, f);

    mTalon.setNeutralMode(NeutralMode.Coast);

    mTalon.config_IntegralZone(PID_LOOP_INDEX, 200, TIMEOUT_MS);

    if (!Robot.getIsPracticeBot()) mTalon.setInverted(true);

    // Set to Slave Mode
    mTalonSlave.follow(mTalon);
  }

  public int getVelocity() {
    return -mTalon.getSelectedSensorVelocity();
  }

  public double getCurrent() {
    return mTalon.getStatorCurrent();
  }

  void setPercentOutput(double output) {
    // System.out.println(getVelocity() + " velocity at output " + output);
    mTalon.set(ControlMode.PercentOutput, -output);
  }

  void setSpeed(int posPer100Ms) {
    if (posPer100Ms == 0) {
      mTalon.set(ControlMode.PercentOutput, 0);
    } else {
      mTalon.set(ControlMode.Velocity, -posPer100Ms);
    }
  }
}
