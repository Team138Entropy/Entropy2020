package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.Axis;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Side;
import frc.robot.Robot;
import frc.robot.util.LatchedBoolean;

// Main Control Class
// Contains instances of the Driver and Operator Controller

public class OperatorInterface {
  private static OperatorInterface mInstance;

  // Instances of the Driver and Operator Controller
  private final XboxController DriverController;
  private final NykoController OperatorController;
  private LatchedBoolean mBarfLatch = new LatchedBoolean();
  private LatchedBoolean mShootLatch = new LatchedBoolean();
  private LatchedBoolean mSpinUpLatch = new LatchedBoolean();
  private LatchedBoolean mVisionToggle = new LatchedBoolean();

  private boolean autoIntakeOverride = false;

  private boolean mIntakeWasPressedWhenWeLastChecked = false;

  private LatchedBoolean mClimbUpWasPressed;
  private LatchedBoolean mClimbDownWasPressed;
  private LatchedBoolean mClimbStartWasPressed;

  public static synchronized OperatorInterface getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorInterface();
    }
    return mInstance;
  }

  private OperatorInterface() {
    DriverController = new XboxController(Constants.DriverControllerPort);
    OperatorController = new NykoController(Constants.OperatorControllerPort);
    mClimbUpWasPressed = new LatchedBoolean();
    mClimbDownWasPressed = new LatchedBoolean();
    mClimbStartWasPressed = new LatchedBoolean();
  }

  // Driver

  public boolean checkControllers() {
    return DriverController.checkNameAndPort() && OperatorController.checkNameAndPort();
  }

  public double getDriveThrottle() {
    return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
  }

  public double getDriveTurn() {
    return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
  }

  public boolean climbUp() {
    boolean buttonValue = DriverController.getButton(XboxController.Button.Y);
    return buttonValue;
  }

  public boolean climbDown() {
    boolean buttonValue = DriverController.getButton(XboxController.Button.A);
    return buttonValue;
  }

  public boolean climbStart() {
    boolean buttonValue = DriverController.getButton(XboxController.Button.BACK);
    return mClimbStartWasPressed.update(buttonValue);
  }

  public boolean startIntake() {
    if (Robot.isAuto()) {
      if (autoIntakeOverride) {
        autoIntakeOverride = false;
        return true;
      } else {
        return false;
      }
    } else {
      boolean buttonValue = DriverController.getButton(XboxController.Button.RB);
      if (mIntakeWasPressedWhenWeLastChecked && !buttonValue) {
        mIntakeWasPressedWhenWeLastChecked = false;
        return true;
      } else {
        mIntakeWasPressedWhenWeLastChecked = buttonValue;
        return false;
      }
    }
  }

  public void setDriverRumble(boolean toggle) {
    DriverController.setRumble(toggle);
  }

  public boolean isBarf() {
    return mBarfLatch.update(DriverController.getButton(XboxController.Button.START));
  }

  // if we are auto steering
  // WHILE held
  public boolean getFeederSteer() {
    return DriverController.getTrigger(Side.RIGHT) || DriverController.getTrigger(Side.LEFT);
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

      } else {
      }
      LowGear = !LowGear;
    }

    // if lowgear value has checked
    DriverController.setRumble(LowGear);
    return LowGear;
  }

  // Operator
  public boolean getVisionToggle() {
    return mVisionToggle.update(OperatorController.getButton(NykoController.Button.BUTTON_3));
  }

  public boolean getShoot() {
    return mShootLatch.update(OperatorController.getButton(NykoController.Button.BUTTON_1));
  }

  public boolean getSpinUp() {
    return mSpinUpLatch.update(OperatorController.getButton(NykoController.Button.BUTTON_4));
  }

  public boolean getStateReset() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_10);
  }

  // Operator trims, etc.

  public boolean getBallCounterAdjustDown() {
    return OperatorController.getDPad() == DPad.LEFT;
  }

  public boolean getBallCounterAdjustUp() {
    return OperatorController.getDPad() == DPad.RIGHT;
  }

  public double getTurretAdjust() {
    return OperatorController.getJoystick(NykoController.Side.LEFT, Axis.X);
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

  public double getOperatorThrottle() {
    return OperatorController.getJoystick(NykoController.Side.LEFT, NykoController.Axis.Y);
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

  // TODO: Decide on climber jog and home buttons
  public double getClimberJogSpeed() {
    return OperatorController.getJoystick(NykoController.Side.LEFT, NykoController.Axis.Y);
  }

  public boolean isClimberTest() {
    return OperatorController.getButton(NykoController.Button.RIGHT_BUMPER);
  }

  public boolean getFunctional() {
    return OperatorController.getButton(NykoController.Button.MIDDLE_10);
  }

  /**
   * Tells {@link #startIntake()} to return true the next time it's called. This is a hack to make
   * auto shooting work. Please don't use it anywhere else. Please.
   */
  public void overrideIntake() {
    autoIntakeOverride = true;
  }

  public void resetOverride() {
    autoIntakeOverride = false;
  }
}
