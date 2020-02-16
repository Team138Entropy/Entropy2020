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

  public boolean getShooterVelocityTrimUp() {
    return OperatorController.getDPad() == DPad.UP;
  }

  public boolean getShooterVelocityTrimDown() {
    return OperatorController.getDPad() == DPad.DOWN;
  }

  public boolean getResetVelocityTrim() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_9);
  }

  // Operator

  public boolean getHarvestMode() {
    return OperatorController.getButton(NykoController.Button.LEFT_TRIGGER);
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

  public boolean startIntake() {
    return OperatorController.getButton(NykoController.Button.BUTTON_1);
  }

  public void setDriverRumble(boolean toggle) {
    DriverController.setRumble(toggle);
  }

  public boolean isBarf() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_11);
  }

  // Test Mode functions
  public boolean isDriveLeftBackTest() {
    return DriverController.getButton(XboxController.Button.A);
  }

  public boolean isDriveLeftFrontTest() {
    return DriverController.getButton(XboxController.Button.X);
  }

  public boolean isDriveRightBackTest() {
    return DriverController.getButton(XboxController.Button.B);
  }

  public boolean isDriveRightFrontTest() {
    return DriverController.getButton(XboxController.Button.Y);
  }

  public boolean isIntakeRollerTest() {
    return OperatorController.getButton(NykoController.Button.BUTTON_1);
  }

  public boolean isStorageRollerBottomTest() {
    return OperatorController.getButton(NykoController.Button.BUTTON_2);
  }

  public boolean isStorageRollerTopTest() {
    return OperatorController.getButton(NykoController.Button.BUTTON_4);
  }

  public boolean isShooterTest() {
    return OperatorController.getButton(NykoController.Button.BUTTON_3);
  }
}
