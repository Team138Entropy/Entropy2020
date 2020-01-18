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
import frc.robot.util.geometry.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

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
    //Zero all nesscary sensors on Robot
    ZeroSensors();

    //Reset Robot State - Note starting position of the Robot
    //This starting Rotation, X, Y is now the Zero Point
    //mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());


  }

  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors(){    
    mSubsystemManager.ZeroSensors();
  }


  public void autonomousInit(){
  }

  public void autonomousPeriodic(){
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
