/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX mTalonMaster = new WPI_TalonSRX(1);
  WPI_TalonSRX mTalonSlave = new WPI_TalonSRX(3);

  XboxController mController = new XboxController(0);

  float value = 1.0f;

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    mTalonSlave.follow(mTalonMaster);
    mTalonMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (mController.getAButtonPressed()) {
      value -= 0.05f;
    }
    if (mController.getBButtonPressed()) {
      value += 0.05f;
    }

    value = Math.min(value, 1.0f);
    value = Math.max(value, 0f);

    System.out.println(
        "Set output value to "
            + value
            + " at velocity "
            + "1: "
            + mTalonMaster.getSensorCollection().getAnalogInVel()
            + " 2: "
            + mTalonMaster.getSensorCollection().getPulseWidthPosition());

    // mTalonMaster.set(ControlMode.PercentOutput, value);
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
