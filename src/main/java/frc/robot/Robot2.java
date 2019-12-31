/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.IO.ControlBoard;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot2 extends IterativeRobot {

  //Controller Reference
  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  //Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  //Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //Subsystem 
  private final Drive mDrive = Drive.getInstance();
  private final VisionManager mVisionManager = VisionManager.getInstance();

  //Variables carried from loop to loop
  boolean mLowGear = false;

  //Control Booleans
  private LatchedBoolean mAutoSteerPressed = new LatchedBoolean();
  private LatchedBoolean mGearShiftPressed = new LatchedBoolean();


  //Sensors
	public ADXRS450_Gyro gyro;
  double kp = .03;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    //Zero all Sensors (inlcuding in Subsystems)
    ZeroSensors();

    //get the gyro started
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    gyro.reset();
  }

  //Call all Sensors to be zeroed
  private void ZeroSensors(){
    mSubsystemManager.ZeroSensors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  //  System.out.println("Robotic Period");
  
  }


  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
   for(int i = 0; i < 100000; i++){
        double angle = gyro.getAngle();
        mDrive.setDrive(.4, -angle * kp, false);
        Timer.delay(0.005);

    }
  }


  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   // System.out.println("TeleopPeriodic Running");
   UserControl();
  }



  @Override
  public void testInit() {
    System.out.println("Test Initiated");

    //Run Subsystem Test Methods


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

 
  }




  //Called mostly during Teleop
  //User Controlled RObot
  public void UserControl(){
    //Poll Common User Input Controls
    double DriveThrottle = mControlBoard.getDriveThrottle();
    double DriveTurn = mControlBoard.getDriveTurn();
    double OperatorThrottle = mControlBoard.getOperatorThrottle();
    double OperatorTurn = mControlBoard.getOperatorTurn();
    mLowGear = mControlBoard.CheckLowGear(mLowGear);

    //Commmands
    boolean auto_steer = mAutoSteerPressed.update(false);



      //System.out.println("DRIVE THROTTLE: " + String.valueOf(DriveThrottle) + "   DRIVE TURN: " + String.valueOf(DriveTurn));
      //System.out.println("OPERATOR THROTTLE: " + String.valueOf(OperatorThrottle) + "   OPERATOR TURN: " + String.valueOf(OperatorTurn));

    boolean quickturn = false;

    //Button call to check if Auto Steering
    boolean AutoSteer = false;

  


    //Call function according to steer mode
    if(AutoSteer == true){
      //Call Autosteer function
    }else{
      //Standard Drive Drive
      mDrive.setDrive(DriveThrottle, DriveTurn, quickturn);
    }

  }


}
