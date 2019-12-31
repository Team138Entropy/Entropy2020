package frc.robot;


/*
    Robote state tracks the robot throughout the match
    

*/
public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private RobotState(){
        
    }



}