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

  //Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  //Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  //Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();

  //Variables from State

  
  static Potentiometer pot;
  public static PotPID potHandler;
  static NetworkTable table;
  private double initialYaw = 0.0;
  private static double targetPos = 50;

  static Logger robotLogger = new Logger("robot"); 
  static Logger visionLogger = new Logger("vision");
  static Logger potLogger = new Logger("pot");

  public static WPI_TalonSRX rotatorTalon;
  private double previousYaw = 0;

  //autonomousInit, autonomousPeriodic, disabledInit, 
  //disabledPeriodic, loopFunc, robotInit, robotPeriodic, 
  //teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    
    //Zero all nesscary sensors on Robot
    ZeroSensors();

    //Reset Robot State
    //Wherever the Robot is now is the starting position
    mRobotState.reset();

    
    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SmartDashboard");

    pot = new AnalogPotentiometer(Config.getInstance().getInt(Key.ROBOT__POT__LOCATION), Config.getInstance().getFloat(Key.ROBOT__POT__RANGE), Config.getInstance().getFloat(Key.ROBOT__POT__OFFSET));
    rotatorTalon = new WPI_TalonSRX(Config.getInstance().getInt(Key.ROBOT__TURRET__TALON_LOCATION));
    potHandler = new PotPID(pot);
  }

  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors(){
    mSubsystemManager.ZeroSensors();
  }

  @Override
  public void autonomousInit(){
  }
  @Override
  public void autonomousPeriodic(){
    rotatorTalon.set(ControlMode.PercentOutput, 0.05f);
  }

  @Override
  public void teleopInit() {
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
    potHandler.disable();
    Config.getInstance().reload();
  }
  @Override
  public void disabledPeriodic(){
    robotLogger.info("pot " + Robot.pot.get());
  }


  /*  
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop(){

    float potMin = Config.getInstance().getFloat(Key.OI__VISION__POT__MIN);
    float potMax = Config.getInstance().getFloat(Key.OI__VISION__POT__MAX);

    boolean allowMovement = (pot.get() < potMax && pot.get() > potMin) || true;
    potLogger.debug("allow movement " + allowMovement);
    
    if(allowMovement){
      if(Config.getInstance().getBoolean(Key.OI__VISION__ENABLED)){
        // vision goes here
      }else{
        // visionLogger.verbose("Not enabled " + targetPos);
        potHandler.enable();
        potHandler.setSetpoint(targetPos);
        if(OperatorInterface.getInstance().getTurretAdjustLeft()) targetPos -= 2.5;
        if(OperatorInterface.getInstance().getTurretAdjustRight()) targetPos += 2.5;
      }
    }else{
      
    }

      
    if(Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)){
      //Check User Inputs
      double DriveThrottle = mOperatorInterface.getDriveThrottle();
      double DriveTurn = mOperatorInterface.getDriveTurn();
      boolean AutoDrive = false;
    }else{
      //Check User Inputs
      double DriveThrottle = mOperatorInterface.getDriveThrottle();
      double DriveTurn = mOperatorInterface.getDriveTurn();
      boolean AutoDrive = false;
    }
  }




  private volatile boolean m_exit;

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

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
