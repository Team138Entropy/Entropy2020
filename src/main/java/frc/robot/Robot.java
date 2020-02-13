package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.events.EventWatcherThread;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import frc.robot.util.geometry.*;
import frc.robot.vision.AimingParameters;
import frc.robot.util.loops.Looper;
import frc.robot.util.Util;
import java.util.Optional;


public class Robot extends TimedRobot {

  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Robot Tracker and Robot Update Tracker
  // Robot Tracker should never be messed with, runs on constructor
  private final RobotTracker mRobotTracker = RobotTracker.getInstance();
  private final RobotTrackerUpdater mRobotTrackerUpdater = RobotTrackerUpdater.getInstance();

  //Loopers
  //Less important task running systems
  private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  // Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Storage mStorage = Storage.getInstance();
  private final Drive mDrive = Drive.getInstance();
  private Turret2 mTurret = Turret2.getInstance();
  private BallIndicator mBallIndicator;
  private CameraManager mCameraManager;

  private static NetworkTable mTable;

  // Relays
  public Relay visionLight = new Relay(0); //Controls Vision Light

  // Robot States
  // each state is small and simple
  enum RobotState {
    Sharpshooter,
    Rebounder,
    Climber
  };

  enum AimingState {
    AutoAim,
    ManualAim,
    Disabled
  };

  enum DrivingState {
    ManualDrive,
    AutoSteer,
    Disabled
  };

  enum ClimbingState {
    Enabled,
    Disabled
  };

  enum IntakingState {
    Enabled,
    Disabled
  };

  enum ShootingState {
    Enabled,
    Disabled
  };

  //Intialize Default States
  private RobotState mRobotState = RobotState.Sharpshooter;
  private AimingState mAimState = AimingState.AutoAim;
  private DrivingState mDriveState = DrivingState.ManualDrive;
  private ClimbingState mClimbState = ClimbingState.Disabled;
  private IntakingState mIntakeState = IntakingState.Disabled;
  private ShootingState mShootingState = ShootingState.Disabled;

  //Latched Booleans
  //Used for detecting presses with debouncing
  //so we accidently don't trigger a press event twice 
  private LatchedBoolean mIntakeTogglePressed = new LatchedBoolean();
  private LatchedBoolean mShootTogglePressed = new LatchedBoolean();
  private LatchedBoolean mRobotModeTogglePressed = new LatchedBoolean();
  private LatchedBoolean mAutoAimTogglePressed = new LatchedBoolean();

  //Auto Steer Aiming Parameters
  //Updated with aiming parameters from robot state
  private Optional<AimingParameters>  mBall_aiming_parameters;

  private final Logger mRobotLogger = new Logger("robot");

  // autonomousInit, autonomousPeriodic, disabledInit,
  // disabledPeriodic, loopFunc, robotInit, robotPeriodic,
  // teleopInit, teleopPeriodic, testInit, testPeriodic
  @Override
  public void robotInit() {
    mRobotLogger.log("Robot Intitialized");

    //Register the Enabled Looper
    //Used to run background tasks!
    //Constantly collects information
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);

    // Zero all nesscary sensors on Robot
    ZeroSensors();

    //Enabled Vision Light
    visionLight.set(Relay.Value.kForward);

    EventWatcherThread.getInstance().start();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");
    mCameraManager = CameraManager.getInstance();
    mCameraManager.init();
    
    // Reset Robot State
    // Wherever the Robot is now is the starting position
    // This starting Rotation, X, Y is now the Zero Point
    mRobotTracker.reset();   
  }


  /*
    Called on bootup, Zero all Sensors
  */
  private void ZeroSensors() {
    mRobotLogger.log("Begining Sensor Zeroing");
    mSubsystemManager.zeroSensors();
    mRobotLogger.log("Sensor Zeroing Complete");
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("BallCounter", "BallValue" + mStorage.getBallCount());
    // TODO: change this to the real boolean
    SmartDashboard.putBoolean("ShooterFull", false);
    // TODO: decide if this is necessary and hook it up
    SmartDashboard.putBoolean("ShooterLoaded", false);
    SmartDashboard.putBoolean("ShooterSpunUp", mShooter.isAtVelocity());
    // TODO: also, finally, hook this up.
    SmartDashboard.putBoolean("TargetLocked", false);
    // TODO: haha that was a joke this is the real last one
    SmartDashboard.putNumber("ElevateTrim", 0.0f);



    // TODO: cameras will go here eventually
  }

  @Override
  public void autonomousInit() {
    mEnabledLooper.start(); //begin background tasks
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    //Start background looper
    //collections information periodically
    mEnabledLooper.start();


  }

  @Override
  public void teleopPeriodic() {
    try {
      RobotLoop();
    } catch (Exception e) {
      mRobotLogger.log("RobotLoop Exception: " + e.getMessage());
      // print the exception to the system error
      e.printStackTrace(System.err);
    }
  }


  @Override
  public void disabledInit() {

    //Turn off background task
    mEnabledLooper.stop();
  }

  @Override
  public void disabledPeriodic() {

  }

 

  


  /*
    Robot Loop
    Main functionality 

    Sharpshooter - 
      Normal mode of operation
      Picking up Balls, then shooting

      this mode of operation is designed for full field shooting

    Rebounder - 
      we know there is going to be enough time inbetween picking up balls
      and we will already be in range

      this mode of operation is designed for rebounding balls in close range
      this mode may not be a good fit, we will find out 

    Climber -
      *UNLESS WE CAN SHOOT AND CLIMB, then this needs to be changed*
      disables rotation on the turret, turns off intake rollers

  */
  public void RobotLoop(){
    //Checks if the overall Robot State Mode wants to change
    //if we change it we perform a bunch of other logic 
    CheckRobotMode(); 
    
    if(mRobotState == RobotState.Sharpshooter){
      //Normal Robot Loop
      //Can't intake, shoot at the same time
      //intake and then shoot
      intakeLoop();
      shootLoop();
    }else if(mRobotState == RobotState.Rebounder){
      //Rebounder
      //run everything!
      //either driver needs to be smart enough to not enable with balls
      //or delay on storage needs to be implimented
      mShooter.start();
      mIntake.start();
      mStorage.start();
    }else if(mRobotState == RobotState.Climber){
      //Climber
      //keep unnesccary subsystems disabled
      //expescially the turret, we don't want that swining around
    }


    //Turret and Drive are indepdent of intake and shoot
    turretLoop();
    driveLoop();
  }

  //Check for a change in Robot State
  //robot is likely always in sharpshooter mode
  public void CheckRobotMode(){
    boolean getTogglePress = mOperatorInterface.ToggleRobotMode();
    boolean WantsToggle = mRobotModeTogglePressed.update(getTogglePress);

    //check if state needs to be reset
    //this is for toggling in between sharpshooter <--> rebounder
    if(WantsToggle){
      if(mRobotState == RobotState.Sharpshooter){
        //go to rebound mode
        mRobotState = RobotState.Rebounder;
      }else if(mRobotState == RobotState.Rebounder){
        //disable all running elements from being in rebounder
        mShooter.stop();
        mIntake.stop();
        mStorage.stop();

        //reset states of individual systems
        mIntakeState = IntakingState.Disabled;
        mAimState = AimingState.AutoAim;

        //go to sharpshooter mode
        mRobotState = RobotState.Sharpshooter;
      }
    }
  }


  /*
    Shoot Loop
    Allows the driver to enable/disable shooting
  */
  public void shootLoop(){
    boolean WantShooterToggle = mOperatorInterface.ToggleShooter();
    boolean ShooterTogglePressed = mShootTogglePressed.update(WantShooterToggle);

    if(ShooterTogglePressed){
      //SHooting Toggle Button Pressed
      if(mShootingState == ShootingState.Enabled ){
        //Disabling Shooting State
        System.out.println("Disable Shooting State");
        mShooter.stop();
        mStorage.stop();
      }else if(mShootingState == ShootingState.Disabled){
        //Enable Shooting State
        System.out.println("Enable Shooting State");
        mShootingState = ShootingState.Enabled;
        
        //Disable the Intake
        mIntakeState = IntakingState.Disabled;

        mShooter.start();
      }
    }

    //If Shooting is enabled..
    if(mShootingState == ShootingState.Enabled){
      //poll if we are at velcoity
      //if at velocity run the motor
    }
  }


  /*
    Intake Loop
    Allow the Driver to Enable/Disable the intake
    Enabling the intake will use current detection to move motors
  */
  public void intakeLoop(){
    boolean WantIntakeToggle = mOperatorInterface.ToggleIntake();
    boolean IntakePressed = mIntakeTogglePressed.update(WantIntakeToggle);

    //Update the state of our intake enabled
    if(IntakePressed == true){
      //Intake Button Press Detected
      if(mIntakeState == IntakingState.Disabled){
        //Enable the Intake Subsystem
        mIntakeState = IntakingState.Enabled;
        mIntake.resetOvercurrentCooldown();
        mIntake.start(); //start the roller

      }else if(mIntakeState == IntakingState.Enabled){
        //Disable the Intake Subsystem!
        mIntakeState = IntakingState.Disabled;
        mIntake.stop(); // stop the roller
      }
    }

    //if the intake system is running..
    //check for the overcurrent trigger
    if(mIntakeState == IntakingState.Enabled){
      //check if we have found a ball
      //if so, run the storgae system
      if(mIntake.isBallDetected() == true){
        //tell the storage to jog up
        mStorage.storeBall();

        //incriment ball counter
      }

      //check if store ball should complete
      //the store actuion should be via encoder
      //this is another hacky way to do it
      mStorage.CheckTimer();
    }
  }

  //Controls the Turrets Aiming
  public void turretLoop() {
    boolean WantsAutoAimToggle = mOperatorInterface.ToggleAutoAim();
    boolean AutoAimChangedToggled = mAutoAimTogglePressed.update(WantsAutoAimToggle);

    //Change Aiming State (if pressed)
    if(AutoAimChangedToggled == true){
      if(mAimState == AimingState.AutoAim){
        mAimState = AimingState.ManualAim;
      }else if(mAimState == AimingState.ManualAim){
        mAimState = AimingState.AutoAim;
      }
    }


    switch(mAimState){
      case AutoAim : {
        //Auto Aiming Turret
        //Poll Robot Tracker for targeting information
        RobotTracker.RobotTrackerResult result = mRobotTracker.GetTurretError(Timer.getFPGATimestamp());
        if(result.HasResult){
          //We have Target Information
        }else{
          //No Results, Don't Rotate
        }


        break;
      }
      case ManualAim : {
        System.out.println("Manual Turret Aim");
        //Poll operator for adjustments
        double RotateBy = mOperatorInterface.GetAzmithTurn();

        break;
      }
      case Disabled : {
        //Do Nothing
        break;
      }
    }    
  }


  /*
    Handles all driving related logic
    Manual Drive and Auto Steering to Balls
    Auto Steering should enable intake mode
  */
  private void driveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    //Set to Auto Steer State if wanted
    if(mOperatorInterface.wantsAutoSteer()){
      mDriveState = DrivingState.AutoSteer;
    }else if(mDriveState == DrivingState.AutoSteer){
      //If we are currently in auto steer and no longer need it
      //reset to manual. No need to do this extra
      mDriveState = DrivingState.ManualDrive;
    }


    switch(mDriveState){
      case AutoSteer : {
        //Auto Steering the robot
        //Auto Steers to the ball!
        mBall_aiming_parameters = mRobotTracker.getAimingParameters(false, -1, Constants.kMaxGoalTrackAge);

        //proceed if we have aiming parameters
        //if not continue manual drive
        if(mBall_aiming_parameters.isEmpty() == false){
          //Get Aiming Parameters, and throttle value
          Rotation2d rb = mBall_aiming_parameters.get().getRobotToGoalRotation();
          System.out.println("AutoSteer - Measured Rotation: " + rb.getDegrees());
          mDrive.autoSteer(Util.limit(driveThrottle, 0.3), mBall_aiming_parameters.get());
        }else{
          //No Valid Tracking Packets - Manual Drive
          //We don't have aiming parameters, continue manual drive
          mDrive.setDrive(driveThrottle, driveTurn, false);
        }

        break;
      }
      case ManualDrive : {
        //Drive Controlled Drive
       // System.out.println("Manual Drive");
        mDrive.setDrive(driveThrottle, driveTurn, false);
        break;
      }
      case Disabled : {
        //no driving
        break;
      }
    }; //end of switch statement
  }


  @Override
  public void testInit() {
    mRobotLogger.log("Entropy 138: Test Init");

    mSubsystemManager.checkSubsystems();
  }

  @Override
  public void testPeriodic() {
    // Intake roller ON while button held
    if (mOperatorInterface.isIntakeRollertest()) {
      mIntake.start();
    } else {
      mIntake.stop();
    }

    // Storage rollers ON while button held
    if (mOperatorInterface.isStorageRollerTest()) {
      mStorage.storeBall();
    } else {
      mStorage.stop();
    }

    // Shooter roller ON while button held
    if (mOperatorInterface.isShooterTest()) {
      mShooter.start();
    } else {
      mShooter.stop();
    }
  }



  
}
