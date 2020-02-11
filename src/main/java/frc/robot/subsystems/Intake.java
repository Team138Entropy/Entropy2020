package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Config;
import frc.robot.Config.Key;
import frc.robot.Constants;
/** Add your docs here. */
public class Intake extends Subsystem {

  
  private double SpeedModifier = 1.0;

  private int mOverCurrentCount = 15;
  private final int mOverCurrentCooldownPeriod = 15;
  boolean mInOverCurrent = false;

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
    System.out.println("Intake Start");
    Running = true;
    mRoller.set(ControlMode.PercentOutput, SpeedModifier * Constants.kIntakeRollerSpeed); 
  }

  //returns if the ball is under load
  //we also don't want to return over current too many times
  //so have a cooldown counter
  public boolean isOverCurrent(){
    double current = mRoller.getSupplyCurrent();

    //check counter if we are still in over current
    if(mInOverCurrent == true){
      if(mOverCurrentCount == 0){
        //we reached end of period, reset and allow this to be caught all over agian
        mOverCurrentCount = mOverCurrentCooldownPeriod;
        mInOverCurrent = false;
      }else{
        //keep counting down...
        mOverCurrentCount--;
      }
    }


    //if our current is at the threshold thats considered over current
    if(current >= Constants.kIntakeCurrentThreshold && (mInOverCurrent == false) ){
      mInOverCurrent = true;
      return true;
    }else{
      //no current detection
      return false;
    }
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
