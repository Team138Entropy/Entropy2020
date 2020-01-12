/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.Key;

/**
 * Add your docs here.
 */
public class PotPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  Potentiometer pot;
  public PotPID(Potentiometer pot) {
    // Intert a subsystem name and PID values here
    super("PotPID", Config.getInstance().getDouble(Key.OI__VISION__PID__P), Config.getInstance().getDouble(Key.OI__VISION__PID__I), Config.getInstance().getDouble(Key.OI__VISION__PID__D));
    this.pot = pot;
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    double potValue = Double.parseDouble(String.format("%.2f", this.pot.get()));
    System.out.println("pot value " + potValue);
    return potValue;
  }

  @Override
  protected void usePIDOutput(double output) {
    output = Math.min(output, 0.5d) / 2;
    System.out.println("pid out " + output);
    Robot.rotatorTalon.set(ControlMode.PercentOutput, output);
  }
}
