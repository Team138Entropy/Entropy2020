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

  //Variables from State

  
  static Potentiometer mPot;
  public static PotPID mPotHandler;
  static NetworkTable mTable;
  private double mInitialYaw = 0.0;
  private static double targetPos = 50;

  static Logger sRobotLogger = new Logger("robot"); 
  static Logger sVisionLogger = new Logger("vision");
  static Logger sPotLogger = new Logger("pot");

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

    
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      mPot = new AnalogPotentiometer(Config.getInstance().getInt(Key.ROBOT__POT__LOCATION), Config.getInstance().getFloat(Key.ROBOT__POT__RANGE), Config.getInstance().getFloat(Key.ROBOT__POT__OFFSET));
      sRotatorTalon = new WPI_TalonSRX(Config.getInstance().getInt(Key.ROBOT__TURRET__TALON_LOCATION));
      mPotHandler = new PotPID(mPot);
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

  @Override
  public void autonomousInit(){
    System.out.println("Auto Init Called");
  }
  @Override
  public void autonomousPeriodic(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      sRotatorTalon.set(ControlMode.PercentOutput, 0.05f);
    }
    System.out.println("Auto Periodic");
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

    //Test all Subsystems
    System.out.println("Running Subsystem Checks");
    mSubsystemManager.CheckSubsystems();


  }
  @Override
  public void testPeriodic(){

  }

  @Override
  public void disabledInit() {
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      mPotHandler.disable();
    }
    Config.getInstance().reload();
  }
  @Override
  public void disabledPeriodic(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      sRobotLogger.verbose("pot " + Robot.mPot.get());
    }
  }


  /*  
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop(){
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)){
      float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
      float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);

      boolean allowMovement = (mPot.get() < potMax && mPot.get() > potMin);
      sPotLogger.debug("allow movement " + allowMovement + " because we got " + mPot.get() + " inside of " + potMin + " to " + potMax);
      
      if(allowMovement){
        if(Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)){
          // vision goes here
        }else{
          // visionLogger.verbose("Not enabled " + targetPos);
          mPotHandler.enable();
          mPotHandler.setSetpoint(targetPos);
          if(OperatorInterface.getInstance().getTurretAdjustLeft()) targetPos -= 2.5;
          if(OperatorInterface.getInstance().getTurretAdjustRight()) targetPos += 2.5;
          targetPos = Math.min(Math.max(targetPos, potMin), potMax);
        }
      }else{
        // don't do anything if we're about to break our robot
      }
    }

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
    }else{
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

   // @SuppressWarnings("PMD.CyclomaticComplexity")
  // @Override
  // public void startCompetition() {
  //   robotInit();

  //   // Tell the DS that the robot is ready to be enabled
  //   HAL.observeUserProgramStarting();

  //   while (!Thread.currentThread().isInterrupted() && !m_exit) {
  //     if (isDisabled()) {
  //       m_ds.InDisabled(true);
  //       //disabled();
  //       m_ds.InDisabled(false);
  //       while (isDisabled()) {
  //         m_ds.waitForData();
  //       }
  //     } else if (isAutonomous()) {
  //       m_ds.InAutonomous(true);
  //       //autonomous();
  //       m_ds.InAutonomous(false);
  //       while (isAutonomous() && !isDisabled()) {
  //         m_ds.waitForData();
  //       }
  //     } else if (isTest()) {
  //       LiveWindow.setEnabled(true);
  //       Shuffleboard.enableActuatorWidgets();
  //       m_ds.InTest(true);
  //       //test();
  //       m_ds.InTest(false);
  //       while (isTest() && isEnabled()) {
  //         m_ds.waitForData();
  //       }
  //       LiveWindow.setEnabled(false);
  //       Shuffleboard.disableActuatorWidgets();
  //     } else {
  //       m_ds.InOperatorControl(true);
  //       //teleop();
  //       m_ds.InOperatorControl(false);
  //       while (isOperatorControl() && !isDisabled()) {
  //         m_ds.waitForData();
  //       }
  //     }
  //   }
  // }
}
