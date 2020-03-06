package frc.robot.OI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

/*
    An Xbox Controller Class
*/
public class NykoController {
  private final Joystick mController;

  // Button Enums
  public enum Side {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }

  public enum DPad {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    OTHER
  }

  public enum Button {
    BUTTON_1(1),
    BUTTON_2(2),
    BUTTON_3(3),
    BUTTON_4(4),
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    LEFT_TRIGGER(7),
    RIGHT_TRIGGER(8),
    MIDDLE_9(9),
    MIDDLE_10(10),
    MIDDLE_11(11),
    LEFT_JOYSTICK(12),
    RIGHT_JOYSTICK(13);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }

  // Pass in the port of the Controller
  public NykoController(int portArg) {
    mController = new Joystick(portArg);
    checkNameAndPort();
  }

  public boolean checkNameAndPort() {
    // for some stupid reason, the 300iq people at nyko thought it would be cool and epic to put a
    // tab character after the name of the controller that is reported. the driver station then
    // passes this on to us, so we get something that makes no sense and is bad to debug. is it one
    // space? ten? a tab? something else stupid? we trim the string anyway
    String name = mController.getName().trim();
    if (!name.equals(Constants.OperatorControllerName) || mController.getPort() != 1) {
      DriverStation.reportError(
          "Airflo Controller not found in port 1! Got name "
              + mController.getName()
              + " in port "
              + mController.getPort(),
          new Error().getStackTrace());
      return false;
    }
    return true;
  }

  double getJoystick(Side side, Axis axis) {
    double deadband = Constants.kJoystickThreshold;

    boolean left = side == Side.LEFT;
    boolean y = axis == Axis.Y;
    // multiplies by -1 if y-axis (inverted normally)
    return handleDeadband(
        (y ? -1 : 1) * mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0)), deadband);
  }

  boolean getButton(Button button) {
    return mController.getRawButton(button.id);
  }

  /*
      Xbox Controller D-Pad
      up -> 0, right -> 90, down -> 180, left -> 270
  */
  DPad getDPad() {
    switch (mController.getPOV()) {
      case 0:
        return DPad.UP;
      case 90:
        return DPad.RIGHT;
      case 180:
        return DPad.DOWN;
      case 270:
        return DPad.LEFT;
      default:
        return DPad.OTHER;
    }
  }

  private double handleDeadband(double value, double deadband) {
    return (Math.abs(value) > Math.abs(deadband)) ? value : 0;
  }
}
