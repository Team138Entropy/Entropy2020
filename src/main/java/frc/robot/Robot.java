/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.IO.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  //Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  //Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  //Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final VisionManager mVisionManager = VisionManager.getInstance();

  //Variables from State


  //autonomousInit, autonomousPeriodic, disabledInit, 
  //disabledPeriodic, loopFunc, robotInit, robotPeriodic, 
  //teleopInit, teleopPeriodic, testInit, testPeriodic

  public void robotInit() {
    System.out.println("robot init _ 1");
    
    //Zero all nesscary sensors on Robot
    ZeroSensors();

    //Reset Robot State
    //Wherever the Robot is now is the starting position
    System.out.println("RobotState!");
    mRobotState.reset();
    System.out.println("Robot State Reset");
  }

  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors(){
    System.out.println("Zero");
    mSubsystemManager.ZeroSensors();
    System.out.println("Done Zero");
  }


  public void autonomousInit(){
    System.out.println("Auto Init Called");
  }

  public void autonomousPeriodic(){
    System.out.println("Auto Periodic");
  }

  public void teleopInit() {
    System.out.println("Teleop Init!");
  }

  public void teleopPeriodic() {
    try{
      RobotLoop();
    }catch(Exception e){
      System.out.println("RobotLoop Exception");
    }
  }

  public void testInit() {
    System.out.println("Entropy 138: Test Init");

    //Test all Subsystems
    System.out.println("Running Subsystem Checks");
    mSubsystemManager.CheckSubsystems();


  }

  public void testPeriodic(){

  }


  public void disabledInit() {

  }

  public void disabledPeriodic(){

  }


  /*  
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop(){
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


  }

}
