package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Axis;
import frc.robot.OI.XboxController.Button;

// Main Control Class
// Contains instances of the Driver and Operator Controller
/*
===== Operator =====
Right Stick - Azmith
Start - Up Shooter Speed
Select - Down Shooter Speed
Right Trigger - Manual Shoot



*/

public class OperatorInterface {

  private static OperatorInterface mInstance;

  // Instances of the Driver and Operator Controller
  private final XboxController DriverController;
  private final XboxController OperatorController;

  public static synchronized OperatorInterface getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorInterface();
    }
    return mInstance;
  }

  private OperatorInterface() {
    DriverController = new XboxController(Constants.DriverControllerPort);
    OperatorController = new XboxController(Constants.OperatorControllerPort);
  }

  // === Driver ===
  public double getDriveThrottle() {
    return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
  }

  public double getDriveTurn() {
    return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
  }

  //While Held, the Driver is in Auto Steer Mode 
  //(if we have coordinates) 
  public boolean wantsAutoSteer(){
    return (
      DriverController.getButton(XboxController.Button.LB) || 
      DriverController.getButton(XboxController.Button.RB));
  }

  //Driver attempts to enable/disable intaking
  public boolean ToggleIntake(){
    return DriverController.getButton(XboxController.Button.Y);
  }

  //Todo? Drive Macros

  // === Operator === 

  //Disable/Enable Auto Aim
  //should always stay in auto aim but just in case somehting breaks
  public boolean ToggleAutoAim(){
    return OperatorController.getTrigger(XboxController.Side.LEFT);
  }

  //enables the shooter
  //the shooter will look to clear out all balls in the chamber
  public boolean ToggleShooter(){
    return OperatorController.getTrigger(XboxController.Side.RIGHT);
  }

  //operator has to hit both right back and left back buttons
  public boolean ToggleRobotMode(){
    return OperatorController.getButton(XboxController.Button.LB) && OperatorController.getButton(XboxController.Button.RB);
  }
 
  //allow the operator to prefire shooting
  //allows the operate to spin up the shooter
  public boolean PrefireShooting(){
    return OperatorController.getButton(XboxController.Button.A);
  }

  public double GetAzmithTurn(){
    return OperatorController.getJoystick(XboxController.Side.LEFT, Axis.X);
  }


  //Mode Toggles
  //Toggles for Sharpshooter, Climber, Rebounder
  public boolean GetSharpshooterMode(){
    return OperatorController.getButton(XboxController.Button.START);
  }

  public boolean GetRebounderMode(){
    return OperatorController.getButton(XboxController.Button.BACK);
  }

  public boolean GetClimberMode(){
    return OperatorController.getButton(XboxController.Button.Y);
  }



  public boolean ToggleIntakeDirection(){
    return DriverController.getButton(XboxController.Button.Y);
  }

  public boolean ToggleInnerRollers(){
    return DriverController.getButton(XboxController.Button.X);
  }

  public boolean ToggleInnerRollersDirection(){
    return DriverController.getButton(XboxController.Button.B);
  }




  public boolean getDriveShift() {
    return DriverController.getButton(XboxController.Button.START);
  }

  public boolean getClimb() {
    return DriverController.getButton(XboxController.Button.Y);
  }

  public boolean getQuickturn() {
    return DriverController.getButton(XboxController.Button.RB);
  }


  

  public boolean getTurretAdjustLeft() {
    //return OperatorController.getDPad() == DPad.LEFT;
    return false;
  }

  public boolean getTurretAdjustRight() {
    //return OperatorController.getDPad() == DPad.RIGHT;
    return false;
  }

  // Operator

  public boolean getHarvestMode() {
   // return OperatorController.getButton(NykoController.Button.LEFT_TRIGGER);
   return false;
  }

  public double getOperatorThrottle() {
    //return OperatorController.getJoystick(NykoController.Side.LEFT, NykoController.Axis.Y);
    return 0;
  }

  public double getOperatorTurn() {
    //return OperatorController.getJoystick(NykoController.Side.RIGHT, NykoController.Axis.X);
    return 0;
  }

  public boolean getCameraSwap() {
    //return OperatorController.getButton(NykoController.Button.BUTTON_4);
    return false;
  }

  public boolean getShoot() {
    //return OperatorController.getButton(NykoController.Button.BUTTON_3);
    return false;
  }

  public boolean getLoadChamber() {
    //return OperatorController.getButton(NykoController.Button.BUTTON_1);
    return false;
  }

  public void setDriverRumble(boolean toggle) {
    DriverController.setRumble(toggle);
  }

  // Test Mode functions
  public boolean isIntakeRollertest() {
    //return OperatorController.getButton(NykoController.Button.MIDDLE_9);
    return false;
  }

  public boolean isStorageRollerTest() {
   // return OperatorController.getButton(NykoController.Button.MIDDLE_10);
    return false;
  }

  public boolean isShooterTest() {
    //return OperatorController.getButton(NykoController.Button.MIDDLE_11);
    return false;
  }
}
