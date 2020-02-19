package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Logger;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;


/**
 * This Turret singleton extends WPILIB's PID subsystem via @Overriding methods use enable(),
 * disable() and setSetpoint() to control the PID. loop() should be run every tick
 */
public class Turret {
  private static Turret sInstance;

  private final WPI_TalonSRX mTurretTalon;

  private Timer mTimer = new Timer();




  // the target position (on a scale from 0 to 100)
  private double mManualTargetPos = 50;
  private double mLastSetpoint;
  private final double TicksPerDegree = 140.0;

  public static Turret getInstance() {
    if (sInstance == null) {
      sInstance = new Turret();
    }

    return sInstance;
  }

  /** Set up our talon, logger and potentiometer */
  private Turret() {

   mTurretTalon = new WPI_TalonSRX(21);
   mTurretTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
   mTurretTalon.config_kF(0, 0);
   mTurretTalon.config_kP(0, .7);
   mTurretTalon.config_kI(0, 0);
   mTurretTalon.config_kD(0, 0);
   mTurretTalon.config_IntegralZone(0, 50);
   mTurretTalon.setNeutralMode(NeutralMode.Brake);

   mTimer.reset();
    mTimer.start();




  }

  public void SetOutput(double value){
    mTurretTalon.set(ControlMode.PercentOutput, value);
  }

  public void RotateByDegrees(double angle){

    if(Math.abs(angle) < 1.5){
      return;
    }

    double setpoint = mTurretTalon.getSelectedSensorPosition() + (angle * TicksPerDegree);
    mLastSetpoint = setpoint;
    //System.out.println("Target Setpoint: " + setpoint);

    if(mTimer.hasPeriodPassed(.05)){
      mTurretTalon.set(ControlMode.Position, setpoint);

    }
  }

  public double getSetPoint(){
    return mLastSetpoint;
  }

  public int getPosition(){
    return mTurretTalon.getSelectedSensorPosition();
  }

  public int getVelocity(){
    return mTurretTalon.getSelectedSensorVelocity();
  }

 
}
