/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Config;
import frc.robot.Logger;
import frc.robot.Robot;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;

/**
 * Add your docs here.
 */
public class Turret extends PIDSubsystem {
  private Logger mTurretLogger;
  private WPI_TalonSRX mTurretTalon;
  private Potentiometer mPot;
  private double mManualTargetPos = 50;

  /**
   * Add your docs here.
   */
  public Turret() {
    // Intert a subsystem name and PID values 
    super("Turret", Config.getInstance().getDouble(Key.OI__VISION__PID__P), Config.getInstance().getDouble(Key.OI__VISION__PID__I), Config.getInstance().getDouble(Key.OI__VISION__PID__D));
    mTurretLogger = new Logger("turret");
    mTurretTalon = new WPI_TalonSRX(Config.getInstance().getInt(Key.ROBOT__TURRET__TALON_LOCATION));
    mPot = new AnalogPotentiometer(Config.getInstance().getInt(Key.ROBOT__POT__LOCATION), Config.getInstance().getFloat(Key.ROBOT__POT__RANGE), Config.getInstance().getFloat(Key.ROBOT__POT__OFFSET));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // gets the POT value, rounded to 2 decimal places

    // TODO: is this even needed?
    double potValue = Double.parseDouble(String.format("%.2f", this.mPot.get()));
    mTurretLogger.verbose("pot value " + potValue);
    return potValue;
  }

  @Override
  protected void usePIDOutput(double output) {
    // limit the output to prevent the motor from going too fast
    output = Math.min(output, Config.getInstance().getDouble(Key.OI__VISION__PID__MAX_SPEED));
    mTurretLogger.verbose("pid out " + output);
    mTurretTalon.set(ControlMode.PercentOutput, output);
  }

  public double getPotValue(){
    return mPot.get();
  }
  
  public void loop(){
    float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
    float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);

    boolean allowMovement = (mPot.get() < potMax && mPot.get() > potMin);
    mTurretLogger.debug("allow movement " + allowMovement + " because we got " + mPot.get() + " inside of " + potMin + " to " + potMax);
    
    if(allowMovement){
      if(Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)){
        // vision goes here
      }else{
        // visionLogger.verbose("Not enabled " + targetPos);
        enable();
        setSetpoint(mManualTargetPos);
        if(OperatorInterface.getInstance().getTurretAdjustLeft()) mManualTargetPos -= 2.5;
        if(OperatorInterface.getInstance().getTurretAdjustRight()) mManualTargetPos += 2.5;
        mManualTargetPos = Math.min(Math.max(mManualTargetPos, potMin), potMax);
      }
    }else{
      mTurretLogger.verbose("movement blocked");
    }

  }
}