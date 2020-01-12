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
  static Potentiometer pot = new AnalogPotentiometer(0, 2.7027, 0);
  public static final PotPID potHandler = new PotPID(pot);
  static NetworkTable table;
  private double initialYaw = 0.0;
  
  static Logger robotLogger = new Logger("robot");
  static Logger visionLogger = new Logger("vision");
  static Logger potLogger = new Logger("pot");
  
  public static WPI_TalonSRX rotatorTalon = new WPI_TalonSRX(1);
  private double previousYaw = 0;

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

    //Reset Robot State
    //Wherever the Robot is now is the starting position
    mRobotState.reset();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SmartDashboard");
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

  @Override
  public void teleopPeriodic() {
    System.out.println("teleop");
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
    System.out.println("robotloop");
    if(Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)){
      boolean tapeDetected = table.getEntry("tapeDetected").getBoolean(false);
      double thisYaw = table.getEntry("tapeYaw").getDouble(0.0);

      if(initialYaw == 0){
          initialYaw = thisYaw;
      }

      // thisYaw = thisYaw - initialYaw;
      //70 is the maximum, -70 is the minimum


      //7 is our "deadband"
      //TODO: add to config
      if(Math.abs(thisYaw) > 10 && (thisYaw < -20 || thisYaw > 0) && tapeDetected){
        potHandler.enable();
        previousYaw = thisYaw;

        double setPoint = (thisYaw + 90) / 180;
        potLogger.verbose("targeting set point " + Double.toString(setPoint) + " from pot " + Robot.pot.get() + " at " + thisYaw);
        potHandler.setSetpoint(setPoint);

        visionLogger.verbose("thisYaw " + thisYaw + " tapeDetected " + tapeDetected);
      }else{
        potHandler.disable();
        visionLogger.debug("Not getting any output " + Double.toString(thisYaw) + " " + tapeDetected + " at the POT " + pot.get());
        rotatorTalon.set(ControlMode.PercentOutput, 0f);
      }
    }else{

      visionLogger.debug("Not enabled");
      potHandler.setSetpoint(0.12);

      if(Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)){
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
