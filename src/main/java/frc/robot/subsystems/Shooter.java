package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

/**
 *  Shooter System
 * 
 * 
 */
public class Shooter extends Subsystem {

    private final WPI_TalonSRX mShooterSlave;
    private final WPI_TalonSRX mShooterMaster;

    private static Shooter mInstance;

    public static synchronized Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }


    private Shooter(){
        mShooterMaster = new WPI_TalonSRX(Constants.kShooterMasterTalonPort);
        mShooterSlave = new WPI_TalonSRX(Constants.kShooterSlaveTalonPort);
    }

    @Override
    public void ZeroSensors() {}
  
    @Override
    public void CheckSubsystems() {}


    @Override
    public void SetTestValue(double value){

    }
  
    @Override
    public void ClearTestValue(){}

}