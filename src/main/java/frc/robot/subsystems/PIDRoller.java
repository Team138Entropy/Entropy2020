package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class PIDRoller {

  private static final int PID_LOOP_INDEX = 0;
  private static final int TIMEOUT_MS = 10;

  private WPI_TalonSRX mTalonMaster;
  private WPI_TalonSRX mTalonSlave;

  PIDRoller(int MasterTalonPort, int SlaveTalonPort, double p, double i, double d) {
    super();

    mTalonMaster = new WPI_TalonSRX(MasterTalonPort);
    mTalonSlave = new WPI_TalonSRX(SlaveTalonPort);
    
    //Reset back to factory defaults just in case
    mTalonMaster.configFactoryDefault();
    mTalonSlave.configFactoryDefault();

    //Configure for Encoder Feedback
    mTalonMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_LOOP_INDEX, TIMEOUT_MS);
    mTalonMaster.setSensorPhase(false);

    //Map the slavef
    mTalonSlave.follow(mTalonMaster);

    /*
    // All of this was ripped from the 2019 elevator code
    mTalon.configNominalOutputForward(0, TIMEOUT_MS);
    mTalon.configNominalOutputReverse(0, TIMEOUT_MS);
    mTalon.configPeakOutputForward(1, TIMEOUT_MS);
    mTalon.configPeakOutputReverse(-1, TIMEOUT_MS);

    mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_LOOP_INDEX, TIMEOUT_MS);
    mTalon.setSensorPhase(false);

    mTalon.configAllowableClosedloopError(PID_LOOP_INDEX, 0, TIMEOUT_MS);

    mTalon.config_kP(PID_LOOP_INDEX, p);
    mTalon.config_kI(PID_LOOP_INDEX, i);
    mTalon.config_kD(PID_LOOP_INDEX, d);

    mTalon.setNeutralMode(NeutralMode.Coast);

    mTalon.config_IntegralZone(PID_LOOP_INDEX, 200, TIMEOUT_MS);
    */
  }

  void setSpeed(int posPer100Ms) {
    mTalonMaster.set(ControlMode.Velocity, posPer100Ms);
  }
}
