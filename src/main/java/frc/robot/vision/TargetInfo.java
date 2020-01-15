package frc.robot.vision;


/**
 *  Container Class for Targets detected by the division system
 *  Contains a center point in 3D Space
 * 
 */

 public class TargetInfo {

    protected double x = 0;
    protected double y = 0;
    protected double z = 0;
    
    
    public TargetInfo(){

    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getZ(){
        return z;
    }



 }