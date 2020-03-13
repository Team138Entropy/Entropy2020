  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.*;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX mTalonMaster = new WPI_TalonSRX(6);
  WPI_TalonSRX mTalonSlave = new WPI_TalonSRX(5);

  WPI_TalonSRX mTalonTop = new WPI_TalonSRX(7);
  WPI_TalonSRX mTalonBottom= new WPI_TalonSRX(8);

  XboxController mController = new XboxController(0);

  float value = 1.0f;

  FileWriter writer;

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    mTalonSlave.follow(mTalonMaster);
    mTalonMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
    try {
      writer = new FileWriter("output1.csv");
      System.out.println("OPENED FILE WRITER!");
    } catch (Exception e) {
      System.out.println("EXCEPTION! " + e.getMessage());
    }

    // fgain = (100% x 1023)/ nativeUnits
    // fgain = (100% x 1023)/ 9323
    // 6.6
    // mTalonMaster.config_kF(0, 6.6);
    // mTalonMaster.set(ControlMode.Position, )

  }

  @Override
  public void teleopInit(){
    mTalonMaster.set(ControlMode.PercentOutput, -1.0f);
  }

  @Override
  public void teleopPeriodic() {
    System.out.println(
        Integer.toString(mTalonMaster.getSensorCollection().getQuadratureVelocity())
            + ","
            + Double.toString(mTalonMaster.getSupplyCurrent()));

    SmartDashboard.putNumber("Shooter Speed", -mTalonMaster.getSelectedSensorVelocity());
    
    if(-mTalonMaster.getSelectedSensorVelocity() > 2150){
      mTalonTop.set(ControlMode.PercentOutput, .8f);
      mTalonBottom.set(ControlMode.PercentOutput, .8f);
    }else{
      mTalonTop.set(ControlMode.PercentOutput, 0);
      mTalonBottom.set(ControlMode.PercentOutput, 0);
    }


    try {
      writer.append("\n");
      writer.flush();
    } catch (Exception e) {
      // System.out.println("WRITE EXCEPTION");
      // System.out.println("EXCEPTION!");
    }
    /*
    System.out.println(
      mTalonMaster.getSensorCollection().getQuadratureVelocity() +
    "," + mTalonMaster.getSupplyCurrent());
    */
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  public void turretLoop() {}
}
