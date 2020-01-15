package frc.robot.subsystems;

/**
 *  Track's the Robot's State on the field
 *  Concepts adopted from 254
 * 
 * 
 */
public class RobotStateEstimator extends Subsystem {


    private Drive mDrive = Drive.getInstance();




    public void Update(double timestamp){
        final double left_distance = mDrive.getLeftEncoderDistance();
        final double right_distance = mDrive.getRightEncoderDistance();

        //Calculate Left Driven Delta

        //Calucate Right Driven Delta

        //Get Gyro




    }

}