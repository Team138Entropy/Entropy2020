package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.OI.NykoController.DPad;

// Main Control Class
// Contains instances of the Driver and Operator Controller

public class OperatorInterface {
  Logger mLogger;
  private static OperatorInterface mInstance;

  // Instances of the Driver and Operator Controller
  private final XboxController DriverController;
  private final NykoController OperatorController;

  public static synchronized OperatorInterface getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorInterface();
    }
    return mInstance;
  }

  private OperatorInterface() {
    mLogger = new Logger("oi");
    DriverController = new XboxController(Constants.DriverControllerPort);
    OperatorController = new NykoController(Constants.OperatorControllerPort);
  }

  // Driver

  public double getDriveThrottle() {
    return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
  }

  public double getDriveTurn() {
    return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
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
    return OperatorController.getDPad() == DPad.LEFT;
  }

  public boolean getTurretAdjustRight() {
    return OperatorController.getDPad() == DPad.RIGHT;
  }

  // Operator

  public boolean getHarvestMode() {
    return DriverController.getTrigger(XboxController.Side.LEFT);
  }

  public double getOperatorThrottle() {
    return OperatorController.getJoystick(NykoController.Side.LEFT, NykoController.Axis.Y);
  }

  public double getOperatorTurn() {
    return OperatorController.getJoystick(NykoController.Side.RIGHT, NykoController.Axis.X);
  }

  public boolean getCameraSwap() {
    return OperatorController.getButton(NykoController.Button.BUTTON_4);
  }

  public boolean getShoot() {
    return OperatorController.getButton(NykoController.Button.BUTTON_3);
  }

  public boolean getLoadChamber() {
    return OperatorController.getButton(NykoController.Button.BUTTON_1);
  }

  public void setDriverRumble(boolean toggle) {
    DriverController.setRumble(toggle);
  }

  public boolean isManualOverride() {
    return OperatorController.getButton(NykoController.Button.RIGHT_TRIGGER);
  } 

  public boolean isIntakeRollertest() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_9);
  }

  public boolean isStorageRollerTest() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_10);
  }

  public boolean isShooterTest() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_11);
  }
  


}
