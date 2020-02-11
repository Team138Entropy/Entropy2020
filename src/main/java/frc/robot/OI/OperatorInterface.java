package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.OI.NykoController.DPad;

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
  Logger mLogger;
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
    mLogger = new Logger("oi");
    DriverController = new XboxController(Constants.DriverControllerPort);
    OperatorController = new XboxController(Constants.OperatorControllerPort);
  }

  // Driver
  public double getDriveThrottle() {
    return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
  }

  public double getDriveTurn() {
    return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
  }

  public boolean wantsAutoSteer(){
    return DriverController.getTrigger(XboxController.Side.RIGHT);
  }

  //Driver attempts to enable/disable intake
  public boolean ToggleIntake(){
    return DriverController.getButton(XboxController.Button.A);
  }

  //Toggles shooter and begins to run the shooting system
  public boolean ToggleShooter(){
    return OperatorController.getButton(XboxController.Button.A);
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

  // Returns if we are in low gear, sets to low gear as well
  public boolean CheckLowGear(boolean previous) {
    boolean LowGear = previous;
    // Check if Low Gear is Toggled
    if (DriverController.getButton(XboxController.Button.START)) {
      if (LowGear == false) {
        mLogger.verbose("Y PRESSED ON");

      } else {
        mLogger.verbose("Y PRESSED OFF");
      }
      LowGear = !LowGear;
    }

    // if lowgear value has checked
    DriverController.setRumble(LowGear);
    return LowGear;
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
