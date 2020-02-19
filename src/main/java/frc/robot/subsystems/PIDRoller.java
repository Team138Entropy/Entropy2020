package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class PIDRoller {

  private static final int PID_LOOP_INDEX = 0;
  private static final int TIMEOUT_MS = 10;

  private WPI_TalonSRX mTalon;
  private WPI_TalonSRX mTalonSlave;

  PIDRoller(int talonPort, int talon2Port, double p, double i, double d, double f) {
    super();

    mTalon = new WPI_TalonSRX(talonPort);

    // All of this was ripped from the 2019 elevator code
    mTalon.configNominalOutputForward(0, TIMEOUT_MS);
    mTalon.configNominalOutputReverse(0, TIMEOUT_MS);
    mTalon.configPeakOutputForward(1, TIMEOUT_MS);
    mTalon.configPeakOutputReverse(-1, TIMEOUT_MS);
    // mTalon.configFactoryDefault();

    mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 5);
    mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_LOOP_INDEX, TIMEOUT_MS);
    mTalon.setSensorPhase(false);

    mTalon.configAllowableClosedloopError(PID_LOOP_INDEX, 0, TIMEOUT_MS);

    mTalon.config_kP(PID_LOOP_INDEX, p);
    mTalon.config_kI(PID_LOOP_INDEX, i);
    mTalon.config_kD(PID_LOOP_INDEX, d);
    mTalon.config_kF(PID_LOOP_INDEX, f);

    mTalon.setNeutralMode(NeutralMode.Coast);

    mTalon.config_IntegralZone(PID_LOOP_INDEX, 200, TIMEOUT_MS);

    mTalonSlave = new WPI_TalonSRX(talon2Port);
    mTalonSlave.follow(mTalon);
  }

  int getVelocity() {
    return -mTalon.getSelectedSensorVelocity();
  }

  void setSpeed(int posPer100Ms) {
    if (posPer100Ms == 0) {
      mTalon.set(ControlMode.PercentOutput, 0);
    } else {
      mTalon.set(ControlMode.Velocity, -posPer100Ms);
    }
  }
}
