/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.IO.ControlBoard;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  //Controller Reference
  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  //Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  //Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final VisionManager mVisionManager = VisionManager.getInstance();


  //autonomousInit, autonomousPeriodic, disabledInit, 
  //disabledPeriodic, loopFunc, robotInit, robotPeriodic, 
  //teleopInit, teleopPeriodic, testInit, testPeriodic

  public void robotInit() {
    
    //Zero all nesscary sensors on Robot
    ZeroSensors();

    //Reset Robot State
    //Wherever the Robot is now is the starting position

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
    double DriveThrottle = mControlBoard.getDriveThrottle();
    double DriveTurn = mControlBoard.getDriveTurn();
    boolean AutoDrive = false;


    //Continue Driving 
    if(AutoDrive == true){
      //AutoSteer Functionality
    }else{
      //Standard Manual Drive
      mDrive.setDrive(DriveThrottle, DriveTurn, false);
    }


  }




  private volatile boolean m_exit;

  @SuppressWarnings("PMD.CyclomaticComplexity")
  @Override
  public void startCompetition() {
    robotInit();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    while (!Thread.currentThread().isInterrupted() && !m_exit) {
      if (isDisabled()) {
        m_ds.InDisabled(true);
        //disabled();
        m_ds.InDisabled(false);
        while (isDisabled()) {
          m_ds.waitForData();
        }
      } else if (isAutonomous()) {
        m_ds.InAutonomous(true);
        //autonomous();
        m_ds.InAutonomous(false);
        while (isAutonomous() && !isDisabled()) {
          m_ds.waitForData();
        }
      } else if (isTest()) {
        LiveWindow.setEnabled(true);
        Shuffleboard.enableActuatorWidgets();
        m_ds.InTest(true);
        //test();
        m_ds.InTest(false);
        while (isTest() && isEnabled()) {
          m_ds.waitForData();
        }
        LiveWindow.setEnabled(false);
        Shuffleboard.disableActuatorWidgets();
      } else {
        m_ds.InOperatorControl(true);
        //teleop();
        m_ds.InOperatorControl(false);
        while (isOperatorControl() && !isDisabled()) {
          m_ds.waitForData();
        }
      }
    }
  }

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
