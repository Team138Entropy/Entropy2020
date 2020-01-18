package frc.robot.vision;


/**
 *  Container Class for Targets detected by the division system
 *  Contains a center point in 3D Space
 *
 *  On the field there are 3 Types of Targets,
 *  0 - High Goal
 *  1 - Feeder Station
 *  2 - Ball
 *  
 */

 public class TargetInfo {

    protected double x = 0;
    protected double y = 0;
    protected double z = 0;

    private int TargetType = 0;

    //Distance in Feet Currently
    private double distance = 0;


        
    public TargetInfo(int CameraID, int TargetType, int x, int y, double distance){
        this.TargetType = TargetType;
        this.x = x;
        this.y = y;
        this.distance = distance;

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


    public void SetBallTarget(){
        TargetType = 2;
    }

    public void SetFeederStationTarget(){
        TargetType = 1;
    }

    public void SetHighgoalTarget(){
        TargetType = 0;
    }




 }