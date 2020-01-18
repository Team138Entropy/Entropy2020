/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Config.Key;
import frc.robot.IO.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private Drive mDrive;


  //Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  //Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  //Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();

  //Variables from State

  
  static Potentiometer sPot;
  static PotPID sPotHandler;
  static NetworkTable mTable;
  private double mInitialYaw = 0.0;
  private static double sTargetPos = 50;

  Logger mRobotLogger = new Logger("robot"); 
  Logger mVisionLogger = new Logger("vision");
  Logger mPotLogger = new Logger("pot");

  public static WPI_TalonSRX sRotatorTalon;
  private double mPreviousYaw = 0;

  //autonomousInit, autonomousPeriodic, disabledInit, 
  //disabledPeriodic, loopFunc, robotInit, robotPeriodic, 
  //teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    System.out.println("robot init _ 1");
    
    //Zero all nesscary sensors on Robot
    ZeroSensors();

    //Reset Robot State
    //Wherever the Robot is now is the starting position
    System.out.println("Robot State Reset");
    mRobotState.reset();
    
    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");

    //TODO: remove HAS_TURRET and HAS_DRIVETRAIN
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      sPot = new AnalogPotentiometer(Config.getInstance().getInt(Key.ROBOT__POT__LOCATION), Config.getInstance().getFloat(Key.ROBOT__POT__RANGE), Config.getInstance().getFloat(Key.ROBOT__POT__OFFSET));
      sRotatorTalon = new WPI_TalonSRX(Config.getInstance().getInt(Key.ROBOT__TURRET__TALON_LOCATION));
      sPotHandler = new PotPID(sPot);
    }

    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)){
      mDrive = Drive.getInstance();
    }
  }

  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors(){
    System.out.println("Zero");
    mSubsystemManager.ZeroSensors();
    System.out.println("Done Zero");
  }

  private void updateSmartDashboard(){
    //TODO: set this up for real
    SmartDashboard.putString("BallCounter", "BallValue" + " / 5");
    //TODO: change this to the real boolean
    SmartDashboard.putBoolean("ShooterFull", false);
    //TODO: decide if this is necessary and hook it up
    SmartDashboard.putBoolean("ShooterLoaded", false);
    //TODO: hook this up
    SmartDashboard.putBoolean("ShooterSpunUp", false);
    //TODO: also, finally, hook this up.
    SmartDashboard.putBoolean("TargetLocked", false);

    //TODO: cameras will go here eventually

  }
  
  @Override
  public void autonomousInit(){
    System.out.println("Auto Init Called");
  }
  @Override
  public void autonomousPeriodic(){
    System.out.println("Auto Periodic");
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop Init!");
  }

  @Override
  public void teleopPeriodic() {
    try{
      RobotLoop();
    }catch(Exception e){
      System.out.println("RobotLoop Exception");
    }
  }
  @Override
  public void testInit() {
    System.out.println("Entropy 138: Test Init");
  }
  @Override
  public void testPeriodic(){
  }


  @Override
  public void disabledInit() {
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      sPotHandler.disable();
    }
    Config.getInstance().reload();
  }
  @Override
  public void disabledPeriodic(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      mRobotLogger.verbose("pot " + Robot.sPot.get());
    }
  }

  public void turretLoop(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
      float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);

      boolean allowMovement = (sPot.get() < potMax && sPot.get() > potMin);
      mPotLogger.debug("allow movement " + allowMovement + " because we got " + sPot.get() + " inside of " + potMin + " to " + potMax);
      
      if(allowMovement){
        if(Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)){
          // vision goes here
        }else{
          // visionLogger.verbose("Not enabled " + targetPos);
          sPotHandler.enable();
          sPotHandler.setSetpoint(sTargetPos);
          if(OperatorInterface.getInstance().getTurretAdjustLeft()) sTargetPos -= 2.5;
          if(OperatorInterface.getInstance().getTurretAdjustRight()) sTargetPos += 2.5;
          sTargetPos = Math.min(Math.max(sTargetPos, potMin), potMax);
        }
      }else{
        // don't do anything if we're about to break our robot
      }
    }
  }

  public void driveTrainLoop(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)){
      //Check User Inputs
      double DriveThrottle = mOperatorInterface.getDriveThrottle();
      double DriveTurn = mOperatorInterface.getDriveTurn();
      boolean AutoDrive = false;
      mDrive.setDrive(DriveThrottle, DriveTurn, false);

      
      //Climb
      if (mOperatorInterface.getClimb()) {
        //climb!
      }

      //Quickturn
      if (AutoDrive == false && mOperatorInterface.getQuickturn()) {
        //Quickturn!
      }
    }
  }

  /*  
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop(){
    updateSmartDashboard();

    //Check User Inputs
    double DriveThrottle = mOperatorInterface.getDriveThrottle();
    double DriveTurn = mOperatorInterface.getDriveTurn();
    boolean AutoDrive = false;

    //Continue Driving 
    if(AutoDrive == true){
      //AutoSteer Functionality
      //Used for tracking a ball
    }else{
      //Standard Manual Drive
      mDrive.setDrive(DriveThrottle, DriveTurn, false);
    }
    turretLoop();

    driveTrainLoop();

    //Climb
    if (mOperatorInterface.getClimb()) {
      //climb!
    }

    //Quickturn
    if (AutoDrive == false && mOperatorInterface.getQuickturn()) {
      //Quickturn!
    }

    //Operator Controls
    if (mOperatorInterface.getTurretManual() != -1) {
      //manual turret aim
    }

    //Camera Swap
    if (mOperatorInterface.getCameraSwap()) {
      //Swap Camera!
    }

    //Shoot
    if (mOperatorInterface.getShoot()) {
      //Shoot!
    }

    //Load chamber
    //NOTE: This may or may not be necessary depending on how our sensor pack turns out
    if (mOperatorInterface.getLoadChamber()) {
      //Load chamber!
    }

    mShooter.periodic();
  }
}
