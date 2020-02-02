/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.IO.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.vision.AimingParameters;
import frc.robot.util.LatchedBoolean;
import frc.robot.util.geometry.*;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    // Controller Reference
    private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

    // Robot State
    private final RobotState mRobotState = RobotState.getInstance();

    // Subsystem Manager
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // Subsystems - Call all subsystems here to foce initialization
    private final Drive mDrive = Drive.getInstance();
    private final VisionManager mVisionManager = VisionManager.getInstance(); 
    private final Intake mIntake = Intake.getInstance();
    //private final Shooter mShooter = Shooter.getInstance();
    private final Storage mStorage = Storage.getInstance();

    public Relay visionLight = new Relay(0);
    
    // State Variables
    private boolean mAutoAim = true; //autonomously manage turret rotation
    private boolean mAutoShoot = false; //automously manage shooting
    private boolean mAutoIntake = false; //automously manage ball lineup
    private boolean mIntaking = false; //is instake system running?
    private boolean mShooting = false; //are we shooting? (shooter wheels running)

    //Storage so should never be directly run, needs to be on if we are shooting or intaking
    private boolean mStorageRunning = false; 

    private boolean mClimbMode = false;


    // Control Variables
    private LatchedBoolean mWantsAutoSteer = new LatchedBoolean();
    private LatchedBoolean HarvestAim = new LatchedBoolean();


    // autonomousInit, autonomousPeriodic, disabledInit,
    // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
    // teleopInit, teleopPeriodic, testInit, testPeriodic

    public void robotInit() {
        // Zero all nesscary sensors on Robot
        ZeroSensors();
        visionLight.set(Relay.Value.kForward);

        // Reset Robot State - Note starting position of the Robot
        // This starting Rotation, X, Y is now the Zero Point
        mRobotState.reset();

    }

    /*
      Called on bootup, Zero all Sensors
    */
    private void ZeroSensors() {
        mSubsystemManager.ZeroSensors();
    }

    public void autonomousInit() {}

    public void autonomousPeriodic() {}

    public void teleopInit() {
        System.out.println("Teleop Init!");
    }

    public void teleopPeriodic() {
        try {
            RobotLoop();
        } catch (Exception e) {
            System.out.println("RobotLoop Exception: " + e.getMessage());
        }
    }

    public void testInit() {
        System.out.println("Entropy 138: Test Init");

        // Test all Subsystems
        System.out.println("Running Subsystem Checks");
        mSubsystemManager.CheckSubsystems();
    }

    public void testPeriodic() {}

    public void disabledInit() {}

    public void disabledPeriodic() {}

    /*
      Called constantly, houses the main functionality of robot
    */
    public void RobotLoop() {
        System.out.println("==Teleop Loop==");
        // Check User Inputs
        double DriveThrottle = mOperatorInterface.getDriveThrottle();
        double DriveTurn = mOperatorInterface.getDriveTurn();
        boolean WantsLowGear = false;
        
        //Detect Harvest Mode - Harvest mode is HOLD based
        //Driver must be holding it each frame
        boolean HarvestModePressed = (mOperatorInterface.getDriverLeftTriggerPressed() | mOperatorInterface.getDriverRightTriggerPressed());
        if(HarvestModePressed == false){
            mAutoIntake = false;
        }else{
            mAutoIntake = true;
        }
        
        //Check if Auto Shoot has been pressed
        boolean AutoShootPressed = mOperatorInterface.getAutoShootToggle();
        if(AutoShootPressed){
            //Toggle the value of auto shoot!
            mAutoShoot = !mAutoShoot;
        }


        //Turret Control
        if(mAutoAim == true){
            //Turret is automatically aligning to vision target


        }else{
            //Manual Turret Control
            //(save latency by only polling this if we need to)
            double AsmithThrottle = mOperatorInterface.getOperatorThrottle();
            double ShooterSpeed = mOperatorInterface.getOperatorTurn();

            System.out.println("DEBUG: Manual Turret Mode");


        }


        //Shooter System
        if(mAutoShoot){
            //Auto Shooting State
            //Robot determines if range is shootable, and angle is reachable


        }else{
            //Manual Shoot
            //This is trigger based
            //it can be trigger based, or continuous (scoop and score)
            //trigger based is when we aren't going to be in range
        }
        /*
        if(mShooting == true){

            //Ensure that the storage belts are running


        }*/



        //Drive System
        if(mAutoIntake == true){
            System.out.println("DEBUG: Auto Drive");
            //Driver only has control of throttle
            //Steering is vision based
        }else{
            System.out.println("DEBUG: Manual Drive");
            //Standard Manual Drive
            mDrive.setDrive(DriveThrottle, DriveTurn, false);
        }
    }
}
