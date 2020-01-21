/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Key;
import frc.robot.OI.OperatorInterface;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private Drive mDrive;

    // Controller Reference
    private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

    // Robot State
    private final RobotState mRobotState = RobotState.getInstance();

    // Subsystem Manager
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // Subsystems
    private final VisionManager mVisionManager = VisionManager.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Storage mStorage = Storage.getInstance();

    // Variables from State

    private Turret mTurret;
    static NetworkTable mTable;

    Logger mRobotLogger = new Logger("robot");

    // autonomousInit, autonomousPeriodic, disabledInit,
    // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
    // teleopInit, teleopPeriodic, testInit, testPeriodic
    @Override
    public void robotInit() {
        mRobotLogger.log("robot init _ 1");

        // Zero all nesscary sensors on Robot
        ZeroSensors();

        // Reset Robot State
        // Wherever the Robot is now is the starting position
        mRobotLogger.log("Robot State Reset");
        mRobotState.reset();

        EventWatcherThread.getInstance().start();

        // prepare the network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        mTable = inst.getTable("SmartDashboard");

        // TODO: remove HAS_TURRET and HAS_DRIVETRAIN
        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
            mTurret = Turret.getInstance();
        }

        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)) {
            mDrive = Drive.getInstance();
        }
    }

    /*
      Called on bootup, Zero all Sensors
    */
    private void ZeroSensors() {
        mRobotLogger.log("Zeroing sensors...");
        mSubsystemManager.ZeroSensors();
        mRobotLogger.log("Zeroed sensors");
    }

    private void updateSmartDashboard() {
        // TODO: set this up for real
        SmartDashboard.putString("BallCounter", "BallValue" + " / 5");
        // TODO: change this to the real boolean
        SmartDashboard.putBoolean("ShooterFull", false);
        // TODO: decide if this is necessary and hook it up
        SmartDashboard.putBoolean("ShooterLoaded", false);
        // TODO: hook this up
        SmartDashboard.putBoolean("ShooterSpunUp", false);
        // TODO: also, finally, hook this up.
        SmartDashboard.putBoolean("TargetLocked", false);

        // TODO: cameras will go here eventually

    }

    @Override
    public void autonomousInit() {
        mRobotLogger.log("Auto Init Called");

        Config.getInstance().reload();
    }

    @Override
    public void autonomousPeriodic() {
        mRobotLogger.log("Auto Periodic");
        updateSmartDashboard();
    }

    @Override
    public void teleopInit() {
        mRobotLogger.log("Teleop Init!");

        Config.getInstance().reload();
    }

    @Override
    public void teleopPeriodic() {
        try {
            RobotLoop();
        } catch (Exception e) {
            mRobotLogger.log("RobotLoop Exception");

            // print the exception to the system error
            e.printStackTrace(System.err);
        }
    }

    @Override
    public void testInit() {
        mRobotLogger.log("Entropy 138: Test Init");

        Config.getInstance().reload();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void disabledInit() {
        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
            mTurret.disable();
        }
        Config.getInstance().reload();
    }

    @Override
    public void disabledPeriodic() {
        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
            mRobotLogger.verbose("got pot value of " + mTurret.getPotValue());
        }
    }

    public void turretLoop() {
        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_TURRET)) {
            mTurret.loop();
        }
    }

    public void driveTrainLoop() {
        if (Config.getInstance().getBoolean(Key.ROBOT__HAS_DRIVETRAIN)) {
            // Check User Inputs
            double DriveThrottle = mOperatorInterface.getDriveThrottle();
            double DriveTurn = mOperatorInterface.getDriveTurn();
            boolean AutoDrive = false;
            mDrive.setDrive(DriveThrottle, DriveTurn, false);

            // Quickturn
            if (AutoDrive == false && mOperatorInterface.getQuickturn()) {
                // Quickturn!
            }
        }
    }

    /*
      Called constantly, houses the main functionality of robot
    */
    public void RobotLoop() {
        updateSmartDashboard();

        turretLoop();

        driveTrainLoop();

        mShooter.periodic();
        mStorage.periodic();

        // Climb
        if (mOperatorInterface.getClimb()) {
            // climb!
        }

        // Operator Controls
        if (mOperatorInterface.getTurretManual() != -1) {
            // manual turret aim
        }

        // Camera Swap
        if (mOperatorInterface.getCameraSwap()) {
            // Swap Camera!
        }

        // Shoot
        if (mOperatorInterface.getShoot()) {
            // Shoot!
        }

        // Load chamber
        // NOTE: This may or may not be necessary depending on how our sensor pack turns out
        if (mOperatorInterface.getLoadChamber()) {
            // Load chamber!
        }
    }
}
