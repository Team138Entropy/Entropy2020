/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX mTalon0 = new WPI_TalonSRX(0);
  WPI_TalonSRX mTalon1 = new WPI_TalonSRX(1);

  XboxController mController = new XboxController(0);

  float value = 0.25f;

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (mController.getAButton()) {
      value = 0.25f;
    }
    if (mController.getBButton()) {
      value = 0.5f;
    }
    if (mController.getXButton()) {
      value = 0.75f;
    }
    if (mController.getYButton()) {
      value = 1.0f;
    }

    System.out.println("Set output value to " + value);

    mTalon0.set(ControlMode.PercentOutput, value);
    mTalon1.set(ControlMode.PercentOutput, value);
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
