package frc.robot.IO;

import frc.robot.Constants;

// Main Control Class
// Contains instances of the Driver and Operator Controller
/*
Current Mapping:

    Operator:
        Right Stick - Manual Azmith Control
        Left Stick - Manual Shooter Speed (UP -> Increase, Down -> Decrease)
        Left Trigger - Toggle Auto Aim
        Right Trigger - Toggle Auto Shoot
        A - Minimum Elevator Elevation (Closed Mode)
        B - Level Elevation
        Y - Upper Elevation

    Drive:
        Left Stick - Throttle
        Right Stick - Steering
        Right Trigger - Auto Align (harvest)
        Left Trigger - Auto Align (harvest)
        Start - Toggle Gear
        Select - (Also) Toggle Gear





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


    //check if the value of auto aim toggle button is pressed
    public boolean getAutoAimToggle(){
        return OperatorController.getButton(XboxController.Button.RB);
    }

    //check if the auto shoot toggle button is pressed
    public boolean getAutoShootToggle(){
        return OperatorController.getButton(XboxController.Button.LB);
    }

    public double getDriveThrottle() {
        return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    public double getDriveTurn() {
        return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    // Returns if we are in low gear, sets to low gear as well
    public boolean CheckLowGear(boolean previous) {
        boolean LowGear = previous;
        // Check if Low Gear is Toggled
        if (DriverController.getButton(XboxController.Button.START)) {
            if (LowGear == false) {
                System.out.println("Y PRESSED ON");

            } else {
                System.out.println("Y PRESSED OFF");
            }
            LowGear = !LowGear;
        }

        // if lowgear value has checked
        DriverController.setRumble(LowGear);
        return LowGear;
    }

    public double getOperatorThrottle() {
        return OperatorController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    public double getOperatorTurn() {
        return OperatorController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    public void setOperatorRumble(boolean toggle) {
        OperatorController.setRumble(toggle);
    }

    public void setDriverRumble(boolean toggle) {
        DriverController.setRumble(toggle);
    }

    public boolean getDriverLeftTriggerPressed(){
        return DriverController.getTrigger(XboxController.Side.LEFT);
    }

    public boolean getDriverRightTriggerPressed(){
        return DriverController.getTrigger(XboxController.Side.RIGHT);
    }


}
