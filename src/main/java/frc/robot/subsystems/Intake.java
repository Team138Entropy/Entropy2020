package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
/** Add your docs here. */
public class Intake extends Subsystem {

  
  private double SpeedModifier = 1.0;

  private WPI_TalonSRX mRoller;

  private boolean Running = false;

  private static Intake sInstance;

  public static synchronized Intake getInstance() {
    if (sInstance == null) {
      sInstance = new Intake();
    }
    return sInstance;
  }

  private Intake() {
    mRoller = new WPI_TalonSRX(Constants.kIntakeRollerPort);
    mRoller.configFactoryDefault();
    mRoller.setNeutralMode(NeutralMode.Brake);

  }

  public void start() {
    Running = true;
    mRoller.set(ControlMode.PercentOutput, SpeedModifier * Constants.kIntakeRollerSpeed); 
  
  }

  //Incase spitting the ball back out is needed
  public void invert(){
    if(SpeedModifier == -1){
      SpeedModifier = 1;
    }else{
      SpeedModifier = -1;
    }
  }

  /** Stops the roller. */
  public void stop() {
    Running = false;
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  public synchronized boolean IsRunning(){
    return Running;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem(){}
}
