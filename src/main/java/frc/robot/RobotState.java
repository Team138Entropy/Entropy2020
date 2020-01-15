package frc.robot;


/*
    Robot State handles robot tra king throughout the match
    Handles all rotation positional information


    This code uses the concept of Poses2D, which essentially boils down to an x, y and rotation
    for example 
        when we capture a target: 
            Detected Goal Pose -> X_Goal, Y_Goal, Theta_Goal
            Robot Pose at Capture Time -> X_Old, Y_Old, Theta_Old

        However our robot pose is now likely different. We can compensate for this!
        Our Robot Pose now is -> X_Now, Y_Now, Theta_Now

        To find our Angle to Aim -> Atan2(Y_Goal - Y_Now, X_Goal - X_Now)
        To find our Range -> Sqrt((Y_Goal - Y_now)^2  + (X_Goal - X_now)^2 )

    

*/
public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }


    //Class Variables
    private double DistanceDriven; //Distance the robot drives

    //InterpolatingTreeMaps allow getting points that don't exist
    //Uses interopulating maps to use linear interopulation 
    //Uses a max queue size passed in upon construction
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> Field_To_Vehicle_Map;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> Vehicle_To_Turret_Map;





    private RobotState(){
        
    }

    //Reset Robot State
    //Make note of the Starting Position
    public void reset(){
        DistanceDriven = 0;

        /Vehicle_TO


    }


    public synchronized void ResetDriveDistance(){
        DistanceDriven = 0;
    }

    /*
        Record robot's initial position on field
        This is encapsulated in a Pose2D object
    */
    private void SetIntialFieldToRobot(){

    }



}