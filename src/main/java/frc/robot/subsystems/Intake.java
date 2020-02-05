package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;


/**
 * Intake system controls the front roller on the robot
 * There is a single talon that controls it
 */
public class Intake extends Subsystem {

  private final WPI_TalonSRX mRoller;

  private static Intake sInstance;

  private boolean IsOn = false;

  public static synchronized Intake getInstance() {
    if (sInstance == null) {
      sInstance = new Intake();
    }
    return sInstance;
  }

  private Intake() {
    mRoller = new WPI_TalonSRX(Constants.kIntakeTalonPort);
    mRoller.configFactoryDefault();
  }

  /** Starts the Roller **/
  public synchronized void start() {
    mRoller.set(ControlMode.PercentOutput, Constants.kIntakeRollerPercentOutput);
    IsOn = true;
  }

  /** Stops the roller. */
  public synchronized void stop() {
    mRoller.set(ControlMode.PercentOutput, 0);
    IsOn = false;
  }

  /** Return if the Roller is running */
  public synchronized boolean IsRunning(){
      return IsOn;
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}

     
  @Override
    public void SetTestValue(double value){
      mRoller.set(ControlMode.PercentOutput, value);
    }
  
    @Override
    public void ClearTestValue(){
      mRoller.set(ControlMode.PercentOutput, 0);

    }
}